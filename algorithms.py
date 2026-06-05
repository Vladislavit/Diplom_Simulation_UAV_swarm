# algorithms.py — Алгоритми координації: ройовий інтелект, консенсус, аукціон

import math
import random
import config as cfg


# ==================== ОНОВЛЕННЯ СУСІДІВ ====================

def update_neighbors(drones):
    """Побудова списку сусідів для кожного дрона за радіусом зв'язку."""
    for drone in drones:
        if drone.status == 'lost':
            drone.neighbors = []
            continue
        drone.neighbors = []
        for other in drones:
            if other.id == drone.id or other.status == 'lost':
                continue
            dist = math.hypot(other.x - drone.x, other.y - drone.y)
            if dist <= drone.comm_radius:
                drone.neighbors.append(other)


# ==================== РОЙОВИЙ ІНТЕЛЕКТ (РОЗВІДКА) ====================

def swarm_scout(drone, drones, ew_rect, sector_bounds=None,
                follow_leader=None, follower_slot=None):
    """Вибір наступної зони: розвідка або патрулювання.

    Розвідка (є непокриті зони): бонус за непокриту зону, штраф за
    відстань, відштовхування, штраф за РЕБ-ризик.
    Патрулювання (100% покриття): тільки рівномірний розподіл
    (сильне відштовхування) + невеликий рандом, щоб не застрягати.
    Страхувальник: якщо дрон простоює без цілі >3с — стрибок у
    протилежний квадрант від поточної позиції.

    sector_bounds=(x_start, x_end) — вертикальна смуга свого кластера
    (тільки hybrid). Бонус +SECTOR_BONUS за зону в межах сектора, штраф
    -50 за зону поза ним. Коли весь свій сектор покритий — sector-бонус
    вимикається й дрон скаутить вільно.

    follow_leader=drone — глобальний лідер у стратегії leader_follower.
    follower_slot=idx — порядковий номер цього follower серед живих
    followers. Якщо лідер далі за FOLLOWER_DIST, follower летить у СВІЙ
    слот формації позаду лідера (V/трикутник), а не в зону лідера —
    інакше всі злипаються в лінію. Якщо лідер стоїть — тримає поточну
    зону. Близько до лідера — звичайний swarm_scout.
    """
    if drone.status != 'scouting' or drone.target_zone is not None:
        return

    # leader_follower: летимо у свій слот формації позаду лідера
    if follow_leader is not None and follower_slot is not None:
        dist = math.hypot(follow_leader.x - drone.x,
                          follow_leader.y - drone.y)
        if dist > cfg.FOLLOWER_DIST:
            lspeed = math.hypot(follow_leader.vx, follow_leader.vy)
            if lspeed < 0.01:
                return  # лідер стоїть — тримати поточну зону
            # Напрямок руху лідера і перпендикуляр до нього
            dir_x = follow_leader.vx / lspeed
            dir_y = follow_leader.vy / lspeed
            perp_x, perp_y = -dir_y, dir_x
            # Слот: 3 в ряд по ширині, ряди по 50px позаду
            offset_x = (follower_slot % 3 - 1) * 40    # -40, 0, +40
            offset_y = (follower_slot // 3 + 1) * 50   # 50, 100, ...
            target_x = (follow_leader.x - dir_x * offset_y
                        + perp_x * offset_x)
            target_y = (follow_leader.y - dir_y * offset_y
                        + perp_y * offset_x)
            col = max(0, min(cfg.GRID_COLS - 1, int(target_x // cfg.ZONE_W)))
            row = max(0, min(cfg.GRID_ROWS - 1, int(target_y // cfg.ZONE_H)))
            drone.target_zone = (col, row)
            return

    # Страхувальник проти залипань
    if getattr(drone, 'scout_idle_time', 0.0) > 3.0:
        mid_col = cfg.GRID_COLS // 2
        mid_row = cfg.GRID_ROWS // 2
        cur = drone.get_zone() or (0, 0)
        col_range = range(0, mid_col) if cur[0] >= mid_col else range(mid_col, cfg.GRID_COLS)
        row_range = range(0, mid_row) if cur[1] >= mid_row else range(mid_row, cfg.GRID_ROWS)
        drone.target_zone = (random.choice(list(col_range)),
                             random.choice(list(row_range)))
        drone.scout_idle_time = 0.0
        return

    uncovered_count = sum(
        1 for r in range(cfg.GRID_ROWS)
        for c in range(cfg.GRID_COLS)
        if not drone.local_map[r][c]
    )
    patrol_mode = (uncovered_count == 0)

    # Чи весь свій сектор покритий — тоді sector-бонус вимикаємо
    sector_done = False
    if sector_bounds:
        x_start, x_end = sector_bounds
        col_start = max(0, int(x_start // cfg.ZONE_W))
        col_end = min(cfg.GRID_COLS, int(math.ceil(x_end / cfg.ZONE_W)))
        sector_done = all(
            drone.local_map[r][c]
            for r in range(cfg.GRID_ROWS)
            for c in range(col_start, col_end)
        )

    best_score = -999999.0
    best_zone = None

    for row in range(cfg.GRID_ROWS):
        for col in range(cfg.GRID_COLS):
            zx = col * cfg.ZONE_W + cfg.ZONE_W // 2
            zy = row * cfg.ZONE_H + cfg.ZONE_H // 2
            dist = math.hypot(zx - drone.x, zy - drone.y)

            score = 0.0

            if patrol_mode:
                # Рівномірний розподіл: сильне відштовхування + рандом
                for other in drones:
                    if other.id == drone.id or other.status == 'lost':
                        continue
                    oz = other.get_zone()
                    if oz == (col, row):
                        score -= 120.0
                    if other.target_zone == (col, row):
                        score -= 120.0
                score += random.uniform(-10.0, 10.0)
            else:
                # Звичайна розвідка
                if not drone.local_map[row][col]:
                    score += 100.0
                else:
                    score -= 50.0
                score -= dist * 0.08
                for other in drones:
                    if other.id == drone.id or other.status == 'lost':
                        continue
                    oz = other.get_zone()
                    if oz == (col, row):
                        score -= 80.0
                    if other.target_zone == (col, row):
                        score -= 60.0

            # РЕБ-штрафи: risk_map для всіх, геометрія зони тільки у hybrid
            risk = drone.risk_map[row][col]
            if risk > 0.3:
                score -= risk * 200.0
            if cfg.SCENARIO == 'hybrid' and _zone_overlaps_ew(col, row, ew_rect):
                score -= 10.0 + risk * 250.0

            # Сектор кластера (тільки hybrid). Бонус — за зону у власному
            # вертикальному секторі; коли весь сектор покритий — вимикаємо.
            if sector_bounds and not sector_done:
                x_start, x_end = sector_bounds
                if x_start <= zx < x_end:
                    score += cfg.SECTOR_BONUS
                else:
                    score -= 50.0

            if score > best_score:
                best_score = score
                best_zone = (col, row)

    drone.target_zone = best_zone


# ==================== BOIDS (swarm_only) ====================

def boids_move(drone, drones, dt):
    """Класичний Boids: separation + alignment + cohesion.

    Модифікує drone.vx, drone.vy та зсуває drone.x, drone.y.
    Призначений для стратегії 'swarm_only' замість _move_to_zone.

    - Separation: відштовхування від сусідів у радіусі BOIDS_SEP_RADIUS,
      сила пропорційна (radius - dist).
    - Alignment: підлаштування швидкості до середньої швидкості сусідів
      у межах comm_radius.
    - Cohesion: тяжіння до середньої позиції сусідів у межах comm_radius.

    Швидкість обмежується drone.speed; позиція клампиться в межі карти.
    """
    if drone.status == 'lost':
        return

    # Wander — випадковий імпульс, що оновлюється кожні 2-3с. Розбиває
    # симетрію, щоб рій блукав і не застигав купою при слабкій cohesion.
    drone._wander_timer = getattr(drone, '_wander_timer', 0.0) - dt
    if drone._wander_timer <= 0.0:
        drone._wander_x = random.uniform(-1, 1) * cfg.BOIDS_WANDER
        drone._wander_y = random.uniform(-1, 1) * cfg.BOIDS_WANDER
        drone._wander_timer = random.uniform(2.0, 3.0)

    sep_x, sep_y = 0.0, 0.0
    ali_x, ali_y = 0.0, 0.0
    coh_x, coh_y = 0.0, 0.0
    sep_count = 0
    nb_count = 0

    for other in drones:
        if other.id == drone.id or other.status == 'lost':
            continue
        dx = other.x - drone.x
        dy = other.y - drone.y
        dist = math.hypot(dx, dy)
        # Clamp: при майже однаковій позиції dist→0 і (dx/dist) вибухає.
        # Замість пропуску (через continue дрони злипались) — фіксуємо
        # мінімум, тож separation все одно відштовхує близьку пару.
        if dist < 1.0:
            dist = 1.0

        # Separation: лише ближче за BOIDS_SEP_RADIUS
        if dist < cfg.BOIDS_SEP_RADIUS:
            push = (cfg.BOIDS_SEP_RADIUS - dist) / cfg.BOIDS_SEP_RADIUS
            sep_x -= (dx / dist) * push
            sep_y -= (dy / dist) * push
            sep_count += 1

        # Alignment + Cohesion: у межах comm_radius
        if dist <= drone.comm_radius:
            ali_x += other.vx
            ali_y += other.vy
            coh_x += other.x
            coh_y += other.y
            nb_count += 1

    ax, ay = 0.0, 0.0

    if sep_count > 0:
        ax += (sep_x / sep_count) * cfg.BOIDS_SEP_WEIGHT * drone.speed
        ay += (sep_y / sep_count) * cfg.BOIDS_SEP_WEIGHT * drone.speed

    if nb_count > 0:
        avg_vx = ali_x / nb_count
        avg_vy = ali_y / nb_count
        ax += (avg_vx - drone.vx) * cfg.BOIDS_ALI_WEIGHT
        ay += (avg_vy - drone.vy) * cfg.BOIDS_ALI_WEIGHT

        cx = coh_x / nb_count
        cy = coh_y / nb_count
        ax += (cx - drone.x) * cfg.BOIDS_COH_WEIGHT * 0.5
        ay += (cy - drone.y) * cfg.BOIDS_COH_WEIGHT * 0.5

    # Wander як кермова складова (масштаб швидкості, як у sep/ali/coh),
    # сталий 2-3с → плавне меандрування, а не тремтіння.
    ax += drone._wander_x * drone.speed
    ay += drone._wander_y * drone.speed

    # Інтегруємо прискорення
    drone.vx += ax * dt
    drone.vy += ay * dt

    # Обмеження швидкості
    speed = math.hypot(drone.vx, drone.vy)
    if speed > drone.speed:
        drone.vx = drone.vx / speed * drone.speed
        drone.vy = drone.vy / speed * drone.speed

    drone.x += drone.vx * dt
    drone.y += drone.vy * dt

    # Кламп до меж карти
    drone.x = max(12, min(cfg.MAP_WIDTH - 12, drone.x))
    drone.y = max(12, min(cfg.WINDOW_HEIGHT - 12, drone.y))


def _zone_overlaps_ew(col, row, ew_rect):
    """Перевірка перетину зони з РЕБ регіоном (без pygame)."""
    if not ew_rect:
        return False
    ex, ey, ew_w, eh = ew_rect
    zx = col * cfg.ZONE_W
    zy = row * cfg.ZONE_H
    return (zx < ex + ew_w and zx + cfg.ZONE_W > ex and
            zy < ey + eh and zy + cfg.ZONE_H > ey)


# ==================== КОНСЕНСУСНИЙ АЛГОРИТМ ====================

def _merge_neighbor_state(drone, neighbor, fields=('map', 'targets', 'risk')):
    """Спільний merge для intra/inter консенсусу.

    fields обмежує, що саме передається:
      'map'     — local_map (покриття)
      'targets' — known_targets (gossip)
      'risk'    — risk_map (максимум)
    Повертає короткий рядок зі статистикою змін (або '').
    """
    merged_zones = 0
    new_targets = 0

    if 'map' in fields or 'risk' in fields:
        for row in range(cfg.GRID_ROWS):
            for col in range(cfg.GRID_COLS):
                if 'map' in fields:
                    if (neighbor.local_map[row][col]
                            and not drone.local_map[row][col]):
                        drone.local_map[row][col] = True
                        merged_zones += 1
                if 'risk' in fields:
                    if neighbor.risk_map[row][col] > drone.risk_map[row][col]:
                        drone.risk_map[row][col] = neighbor.risk_map[row][col]

    if 'targets' in fields:
        for tid, tinfo in neighbor.known_targets.items():
            if tid not in drone.known_targets:
                drone.known_targets[tid] = tinfo.copy()
                new_targets += 1

    parts = []
    if merged_zones:
        parts.append(f"+{merged_zones} зон")
    if new_targets:
        parts.append(f"+{new_targets} цілей")
    return ", ".join(parts)


def intra_cluster_consensus(drones, log_callback):
    """L1: обмін всередині кластерів (local_map + known_targets + risk_map).

    Тільки між дронами одного кластера (cluster_id збігається). Передумова —
    compute_clusters() уже виставив cluster_id. Дрони поза кластером
    (-1, наприклад attacking/lost) не беруть участі.

    Лог: [C{id}-SYNC], не більше 2 повідомлень на кластер за раунд.
    """
    clusters = {}
    for d in drones:
        if d.status == 'lost' or d.cluster_id < 0:
            continue
        clusters.setdefault(d.cluster_id, []).append(d)

    for cid, members in clusters.items():
        if len(members) < 2:
            continue
        logged = 0
        for drone in members:
            for neighbor in members:
                if neighbor.id == drone.id:
                    continue
                change = _merge_neighbor_state(drone, neighbor)
                if change and logged < 2:
                    log_callback(
                        f"[C{cid}-SYNC] D{neighbor.id}→D{drone.id}: "
                        f"{change}",
                        'consensus')
                    logged += 1


def inter_cluster_consensus(drones, log_callback):
    """L2: лідери кластерів обмінюються known_targets.

    Лідери в межах comm_radius один одного обмінюються відомими цілями
    (двосторонньо). Карта покриття і ризику тут НЕ передаються — для
    глобальної координації важливі лише цілі.

    Передані цілі осідають у leader.known_targets; на наступному тіку
    intra_cluster_consensus розповсюдить їх по всіх членах кластера.

    Лог: [INTER-SYNC] L{a}↔L{b}: +N цілей.
    """
    leaders = [d for d in drones
               if d.is_leader and d.status != 'lost' and d.cluster_id >= 0]

    for i, la in enumerate(leaders):
        for lb in leaders[i + 1:]:
            dist = math.hypot(la.x - lb.x, la.y - lb.y)
            if dist > la.comm_radius:
                continue

            new_to_a = 0
            for tid, tinfo in lb.known_targets.items():
                if tid not in la.known_targets:
                    la.known_targets[tid] = tinfo.copy()
                    new_to_a += 1
            new_to_b = 0
            for tid, tinfo in la.known_targets.items():
                if tid not in lb.known_targets:
                    lb.known_targets[tid] = tinfo.copy()
                    new_to_b += 1

            total = new_to_a + new_to_b
            if total:
                log_callback(
                    f"[INTER-SYNC] L{la.cluster_id}↔L{lb.cluster_id}: "
                    f"+{total} цілей",
                    'consensus')


# --- Старий плоский consensus_step (deprecated, замінено двома вище) ---
def _deprecated_consensus_step(drones, log_callback):
    """Обмін картами покриття та інформацією між сусідами.

    Кожен дрон:
    1. Транслює сусідам свою карту покриття і виявлені цілі
    2. Отримує карти сусідів і об'єднує (merge)
    3. Якщо нова інформація — транслює далі (gossip protocol)
    """
    messages = []

    for drone in drones:
        if drone.status == 'lost':
            continue

        for neighbor in drone.neighbors:
            merged_zones = []

            # Об'єднання карт покриття
            for row in range(cfg.GRID_ROWS):
                for col in range(cfg.GRID_COLS):
                    if neighbor.local_map[row][col] and not drone.local_map[row][col]:
                        drone.local_map[row][col] = True
                        merged_zones.append((col, row))

            # Обмін інформацією про цілі (gossip)
            new_targets = False
            for tid, tinfo in neighbor.known_targets.items():
                if tid not in drone.known_targets:
                    drone.known_targets[tid] = tinfo.copy()
                    new_targets = True

            # Синхронізація карти ризику (максимум)
            for row in range(cfg.GRID_ROWS):
                for col in range(cfg.GRID_COLS):
                    if neighbor.risk_map[row][col] > drone.risk_map[row][col]:
                        drone.risk_map[row][col] = neighbor.risk_map[row][col]

            # Лог про злиття
            if merged_zones:
                zone = merged_zones[0]
                msg = (f"[D{neighbor.id}\u2192D{drone.id}] "
                       f"консенсус: зона {zone[0]},{zone[1]} покрита")
                if msg not in messages:
                    messages.append(msg)

            if new_targets:
                msg = (f"[D{neighbor.id}\u2192D{drone.id}] "
                       f"нова інформація про цілі")
                if msg not in messages:
                    messages.append(msg)

    # Обмеження кількості повідомлень у логу за раунд
    for msg in messages[:3]:
        log_callback(msg, 'consensus')


# ==================== АУКЦІОННИЙ АЛГОРИТМ ====================

def run_auction(drones, target, risk_map_global, log_callback):
    """Призначення дронів на класифіковану ціль за аукціоном.

    bid = w1*(1/відстань) + w2*заряд + w3*зв'язок - w4*перешкоди - w5*ризик

    Топ-N дронів з найвищим bid отримують статус attacking.
    """
    if target.state != 'classified':
        return None

    # Імовірнісна модель: шлемо ОДИН дрон за раз. Поки призначений дрон у
    # польоті — нового не шлемо; його промах звільнить ціль для наступного.
    already_attacking = sum(
        1 for d in drones
        if d.status == 'attacking' and d.target_obj is target
    )
    needed = 1 - already_attacking
    if needed <= 0:
        return None

    # Збір кандидатів та обчислення bid
    max_dist = math.hypot(cfg.MAP_WIDTH, cfg.WINDOW_HEIGHT)
    candidates = []

    for drone in drones:
        # Тільки вільні дрони з достатнім зарядом
        if drone.status != 'scouting':
            continue
        if drone.battery < 15:
            continue
        # Не призначати того ж дрона двічі на ту саму ціль
        if drone.id in target.assigned_drones:
            continue

        # 1. Зворотна відстань (нормалізована)
        dist = math.hypot(drone.x - target.x, drone.y - target.y)
        dist_score = 1.0 - (dist / max_dist)

        # 2. Заряд батареї (нормалізований)
        battery_score = drone.battery / 100.0

        # 3. Зв'язок — кількість сусідів (нормалізовано до 1.0)
        connectivity = min(len(drone.neighbors) / 5.0, 1.0)

        # 4. Перешкоди (спрощено — 0)
        obstacles = 0.0

        # 5. Ризик зони цілі
        tz = target.get_zone()
        risk = 0.0
        if tz:
            risk = risk_map_global[tz[1]][tz[0]]

        bid = (cfg.W_DISTANCE * dist_score +
               cfg.W_BATTERY * battery_score +
               cfg.W_CONNECTIVITY * connectivity -
               cfg.W_OBSTACLES * obstacles)

        # Ризик зони цілі. У hybrid — дворівневий мультиплікативний штраф
        # за рівнем впевненості в РЕБ: 0.3–0.6 (підозра) → половина bid;
        # >0.6 (підтверджено) → bid у ~10 разів менший, рій перемикається
        # на інші цілі. У решті стратегій — класичний адитивний W_RISK.
        if cfg.SCENARIO == 'hybrid':
            if risk > 0.6:
                bid *= 0.1   # підтверджена РЕБ — майже ігнорувати ціль
            elif risk > 0.3:
                bid *= 0.5   # підозра на РЕБ — половинний пріоритет
        else:
            bid -= cfg.W_RISK * risk

        # Додатковий штраф для танків (висока небезпека для дрона)
        if target.type == 'tank':
            bid *= cfg.W_TANK_PENALTY

        drone.bid = round(bid, 2)
        candidates.append((drone, bid))

    # Сортування за bid — від найвищого
    candidates.sort(key=lambda x: x[1], reverse=True)

    # Перевірка порогу: якщо переможець занадто слабкий — fallback або пропуск
    if candidates and candidates[0][1] < cfg.MIN_BID_THRESHOLD:
        fallback = next(
            (d for d, _ in candidates
             if getattr(d, 'scout_idle_time', 0.0) > cfg.FALLBACK_SCOUT_TIME),
            None
        )
        if fallback:
            fallback.status = 'attacking'
            fallback.target_obj = target
            fallback.target_zone = None
            target.assigned_drones.append(fallback.id)
            log_callback(
                f"[FALLBACK] D{fallback.id} → {target.label} "
                f"(bid<{cfg.MIN_BID_THRESHOLD:.2f}, idle>{cfg.FALLBACK_SCOUT_TIME:.0f}s)",
                'warning'
            )
            if target.hits_received >= target.drones_needed:
                target.auction_done = True
            return {
                'assigned': [fallback],
                'all_bids': [(d.id, b) for d, b in candidates],
                'winner_ids': [fallback.id],
            }
        else:
            log_callback(
                f"[LOW-PRI skip] {target.label} — bid={candidates[0][1]:.2f}",
                'warning'
            )
            return None

    # Призначення топ-N дронів
    assigned = []
    bid_parts = []

    for drone, bid in candidates[:needed]:
        drone.status = 'attacking'
        drone.target_obj = target
        drone.target_zone = None
        target.assigned_drones.append(drone.id)
        assigned.append(drone)
        bid_parts.append(f"D{drone.id}={bid:.2f}")

    if assigned:
        bids_str = ", ".join(bid_parts)
        log_callback(
            f"[AUCTION] {bids_str} \u2192 {target.label} (bid)",
            'auction'
        )

    # Знищення тепер імовірнісне (у drone._move_to_target), тож тут
    # auction_done не чіпаємо — ціль закриється сама при state='destroyed'.

    # Повертаємо повні дані аукціону для метрик
    return {
        'assigned': assigned,
        'all_bids': [(d.id, b) for d, b in candidates],
        'winner_ids': [d.id for d in assigned],
    }


# ==================== КЛАСТЕРИЗАЦІЯ РОЮ ====================

def compute_clusters(drones):
    """Переобрання лідерів і live-центроїди (статичні кластери у hybrid).

    Кластери призначаються один раз у Simulation.reset() через
    drone.cluster_id (initial_cluster_id), і ця функція їх НЕ
    перебудовує. Кожного тіку вона:

    1. Скидає is_leader для всіх дронів.
    2. Для кожного cluster_id серед живих дронів робить лідером того,
       у кого найменший id → автоматичне переобрання при втраті лідера.
    3. Обчислює центроїд живих членів кожного кластера для візуалізації.

    Дрони зі status == 'lost' пропускаються (не входять у центроїд, не
    можуть бути лідером).

    Повертає {cluster_id: (centroid_x, centroid_y)} лише живих кластерів.
    """
    clusters = {}
    for d in drones:
        d.is_leader = False
        if d.status == 'lost' or d.cluster_id < 0:
            continue
        clusters.setdefault(d.cluster_id, []).append(d)

    centroids = {}
    for cid, members in clusters.items():
        leader = min(members, key=lambda d: d.id)
        leader.is_leader = True
        cx = sum(m.x for m in members) / len(members)
        cy = sum(m.y for m in members) / len(members)
        centroids[cid] = (cx, cy)

    return centroids


def run_cluster_auction(drones, targets, risk_map, sectors,
                        log_callback, metrics_cb=None):
    """Дворівневий аукціон для 'hybrid' — статичні кластери + сектори.

    Передумова: drone.cluster_id виставлений у Simulation.reset() і не
    змінюється; compute_clusters() щотіку оновлює is_leader.

    sectors — словник {cluster_id: (x_start, x_end)} з вертикальними
    смугами карти, прив'язаними до кластерів.

    Рівень 1 (внутрішньокластерний): для кожного кластера беремо цілі,
    чий target.x лежить у секторі цього кластера, і проводимо
    run_auction лише серед його живих scouting-членів. [C{id}-L1].

    Рівень 2 (міжкластерний): цілі поза будь-яким сектором або не взяті
    L1 — глобальний run_auction серед усіх вільних дронів. [INTER-CLUSTER].

    metrics_cb(target, all_bids, winner_ids) — опціональний колбек метрик.
    """
    def noop(*args, **kwargs):
        return None

    # Групуємо живих дронів за статичним cluster_id
    clusters = {}
    for d in drones:
        if d.status == 'lost' or d.cluster_id < 0:
            continue
        clusters.setdefault(d.cluster_id, []).append(d)

    def needs_hit(target):
        if target.state != 'classified':
            return False
        already = sum(
            1 for d in drones
            if d.status == 'attacking' and d.target_obj is target
        )
        return already == 0   # один дрон за раз (імовірнісна модель)

    # ---- Рівень 1: ціль у секторі кластера → внутрішньокластерний ----
    for cid, members in clusters.items():
        sector = sectors.get(cid)
        if not sector:
            continue
        x_start, x_end = sector
        for target in targets:
            if not needs_hit(target):
                continue
            if not (x_start <= target.x < x_end):
                continue
            result = run_auction(members, target, risk_map, noop)
            if result and result['assigned']:
                ids = ", ".join(f"D{d.id}" for d in result['assigned'])
                log_callback(f"[C{cid}-L1] {ids} → {target.label}",
                             'auction')
                if metrics_cb:
                    metrics_cb(target, result['all_bids'],
                               result['winner_ids'])

    # ---- Рівень 2: цілі без сектора або не взяті L1 → глобальний ----
    free_drones = [d for d in drones if d.status == 'scouting']
    for target in targets:
        if not needs_hit(target):
            continue
        result = run_auction(free_drones, target, risk_map, noop)
        if result and result['assigned']:
            ids = ", ".join(f"D{d.id}" for d in result['assigned'])
            log_callback(f"[INTER-CLUSTER] {ids} → {target.label}",
                         'auction')
            if metrics_cb:
                metrics_cb(target, result['all_bids'], result['winner_ids'])


# ==================== ЛІДЕР-ПОСЛІДОВУВАЧ ====================

def run_leader_follower(drones, targets, log_callback):
    """Ієрархічна координація: один лідер призначає цілі follower-дронам.

    На відміну від аукціону, рішення централізоване й без bid: лідер
    бере найближчу до себе класифіковану ціль, що ще потребує ударів,
    і призначає найближчого вільного follower-дрона. Якщо вільних
    follower немає — лідер атакує ціль сам.

    Лідер ЄДИНИЙ і глобальний: зафіксований у Simulation.reset()
    (is_leader=True на дроні з мін. id) і НЕ переобирається. Ця функція
    лідера не призначає — лише читає is_leader. Коли лідер гине,
    leaders стає порожнім, команди не видаються, рій деградує — навмисна
    демонстрація вразливости ієрархії на контрасті з hybrid.

    Консенсус і РЕБ-адаптація для цієї стратегії вимкнені (див. main).
    """
    # Лідер зафіксований у reset(); тут лише читаємо is_leader і
    # відсікаємо загиблого. Жодного переобрання.
    leaders = [d for d in drones if d.is_leader and d.status != 'lost']
    leader_ids = {l.id for l in leaders}

    if not leaders:
        return

    def free_followers():
        """Вільні follower-дрони (не лідери, у розвідці, з зарядом)."""
        return [
            d for d in drones
            if d.id not in leader_ids
            and d.status == 'scouting'
            and d.battery >= 15
        ]

    for leader in leaders:
        # Класифіковані цілі, що ще потребують ударів — обираємо
        # найближчу до цього лідера.
        pending = []
        for target in targets:
            if target.state != 'classified':
                continue
            already = sum(
                1 for d in drones
                if d.status == 'attacking' and d.target_obj is target
            )
            if already > 0:        # дрон уже летить — чекаємо результату
                continue
            dist = math.hypot(leader.x - target.x, leader.y - target.y)
            pending.append((dist, target))

        if not pending:
            continue

        pending.sort(key=lambda p: p[0])
        target = pending[0][1]

        # Найближчий вільний follower, який ще не призначений на ціль
        followers = [
            d for d in free_followers()
            if d.id not in target.assigned_drones
        ]
        if followers:
            attacker = min(
                followers,
                key=lambda d: math.hypot(d.x - target.x, d.y - target.y)
            )
            arrow = f"D{leader.id}→D{attacker.id}"
        elif (leader.status == 'scouting'
              and leader.battery >= 15
              and leader.id not in target.assigned_drones):
            # Вільних follower немає — лідер атакує сам
            attacker = leader
            arrow = f"D{leader.id}(сам)"
        else:
            continue

        attacker.status = 'attacking'
        attacker.target_obj = target
        attacker.target_zone = None
        target.assigned_drones.append(attacker.id)

        log_callback(
            f"[LEADER] {arrow} → {target.label}",
            'auction'
        )
        # Знищення імовірнісне (drone._move_to_target) — auction_done тут
        # не виставляємо; промах сам відкриє ціль для наступного дрона.
