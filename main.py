# main.py — Головний файл: запуск симуляції, основний цикл, оркестрація

import pygame
import sys
import math
import numpy as np
import random

import config as cfg
from drone import Drone, draw_hybrid_mesh
from targets import Target
from algorithms import (update_neighbors, swarm_scout,
                        intra_cluster_consensus, inter_cluster_consensus,
                        run_auction, run_leader_follower,
                        compute_clusters, run_cluster_auction,
                        boids_move)
from map_renderer import MapRenderer
from ui_panel import UIPanel
from metrics import MetricsCollector


class Simulation:
    """Основний клас симуляції рою БПЛА."""

    def __init__(self):
        # Шрифти (pygame вже ініціалізовано)
        self.fonts = {
            'small':  pygame.font.SysFont('arial', 12),
            'tiny':   pygame.font.SysFont('arial', 10),
            'normal': pygame.font.SysFont('arial', 13),
        }
        self.font_pause = pygame.font.SysFont('arial', 20, bold=True)
        self.font_hint = pygame.font.SysFont('arial', 11)
        self.speed_multiplier = 1

        # Рендерери
        self.map_renderer = MapRenderer()
        self.ui_panel = UIPanel()

        # Метрики
        self.metrics = MetricsCollector()

        # Активна стратегія (перемикається клавішами 1/2/3/4)
        self.current_strategy = 'hybrid'

        self._new_game()

    def _new_game(self):
        """Генерація нового seed і повний рестарт."""
        cfg.MAP_SEED = random.randint(0, 99999)
        self.reset()

    def reset(self):
        """Скидання стану симуляції до початкового (з поточним cfg.MAP_SEED)."""
        rng = np.random.RandomState(cfg.MAP_SEED)
        random.seed(cfg.MAP_SEED)

        # РЕБ зона — позиція випадкова per-seed, розмір фіксований.
        # Два виклики rng тут, щоб target-позиції залишались детермінованими.
        ew_x = int(rng.randint(60, cfg.MAP_WIDTH - cfg.EW_W - 60))
        ew_y = int(rng.randint(50, int(cfg.WINDOW_HEIGHT * 0.68) - cfg.EW_H))
        self.ew_rect = (ew_x, ew_y, cfg.EW_W, cfg.EW_H)

        # Зберегти вибрану стратегію між скиданнями; дефолт — 'hybrid'
        self.current_strategy = getattr(self, 'current_strategy', 'hybrid')

        self.sim_time = 0.0
        self.paused = False
        self.selected_drone = None
        self.selected_target = None
        self.mission_complete = False
        self.mission_failed = False
        self._report_generated = False
        self._end_reason = ''
        self.speed_multiplier = getattr(self, 'speed_multiplier', 1)

        # Глобальні карти
        self.global_coverage = [
            [False] * cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)
        ]
        self.risk_map = [
            [0.0] * cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)
        ]
        # Лічильник EW-втрат у кожній зоні — накопичення впевненості в РЕБ:
        # 1 втрата = підозра, 2+ = підтверджена загроза.
        self.loss_count_map = [
            [0] * cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)
        ]

        # Дрони — формація біля бази
        self.drones = []
        for i in range(cfg.NUM_DRONES):
            x = cfg.DRONE_START_X + (i % cfg.DRONE_COLS) * cfg.DRONE_SPACING_X
            y = cfg.DRONE_START_Y + (i // cfg.DRONE_COLS) * cfg.DRONE_SPACING_Y
            drone = Drone(i, x, y)
            # Унікальний початковий вектор — розводить рій по різних
            # напрямках на старті (критично для swarm_only Boids, щоб не
            # злипались; в інших стратегіях одразу перекривається steering).
            angle = (drone.id / cfg.NUM_DRONES) * 2 * math.pi
            drone.vx = math.cos(angle) * cfg.DRONE_SPEED * 0.5
            drone.vy = math.sin(angle) * cfg.DRONE_SPEED * 0.5
            self.drones.append(drone)

        # === Статична кластеризація для hybrid ===
        # Призначаємо за порядковим id: перші CLUSTER_SIZE дронів → cluster 0,
        # наступні → cluster 1 і т.д. Кластер незмінний протягом місії; при
        # смерті лідера compute_clusters() переобирає нового з тим самим cid.
        num_clusters = cfg.NUM_DRONES // cfg.CLUSTER_SIZE
        for drone in self.drones:
            cid = min(drone.id // cfg.CLUSTER_SIZE, num_clusters - 1)
            drone.cluster_id = cid
            drone.initial_cluster_id = cid  # незмінний, навіть при втраті

        # Сектори: рівні вертикальні смуги карти по cluster_id
        sector_width = cfg.MAP_WIDTH / num_clusters
        self.cluster_sectors = {
            cid: (cid * sector_width, (cid + 1) * sector_width)
            for cid in range(num_clusters)
        }
        # Лідери — призначення залежить від стратегії:
        #  • hybrid — по одному лідеру на кластер (для дворівневого
        #    аукціону); compute_clusters переобирає їх щотіку при втраті.
        #  • leader_follower — ОДИН глобальний лідер (мін. id), зафіксований
        #    тут і БЕЗ переобрання. При його загибелі рій лишається без
        #    керування й деградує — навмисна демонстрація вразливости
        #    ієрархії на контрасті з hybrid.
        for d in self.drones:
            d.is_leader = False
        self.cluster_leaders = {}
        if cfg.SCENARIO == 'hybrid':
            for cid in range(num_clusters):
                members = [d for d in self.drones if d.cluster_id == cid]
                leader = min(members, key=lambda d: d.id)
                leader.is_leader = True
                self.cluster_leaders[cid] = leader
        elif cfg.SCENARIO == 'leader_follower':
            global_leader = min(self.drones, key=lambda d: d.id)
            global_leader.is_leader = True

        # Прогрес місії — для умови застою
        self.last_progress_time = 0.0
        self.stagnation_triggered = False

        # Цілі — розподілені по верхній частині карти
        self.targets = []
        types_pool = ['infantry', 'infantry', 'infantry', 'infantry',
                      'vehicle', 'vehicle', 'vehicle',
                      'bmp', 'bmp',
                      'tank']
        # Доповнити до NUM_TARGETS (12)
        while len(types_pool) < cfg.NUM_TARGETS:
            types_pool.append('infantry')
        ex, ey, ew_w, ew_h = self.ew_rect
        for i, ttype in enumerate(types_pool):
            tx = int(rng.randint(60, cfg.MAP_WIDTH - 60))
            ty = int(rng.randint(50, int(cfg.WINDOW_HEIGHT * 0.68)))
            # Танк розміщуємо всередині РЕБ зони — для демонстрації адаптації
            if ttype == 'tank':
                tx = ex + ew_w // 2
                ty = ey + ew_h // 2
            self.targets.append(Target(i, tx, ty, ttype))

        # Таймери консенсусу: intra (часто) і inter (рідше)
        self.consensus_timer = 0.0
        self.inter_consensus_timer = 0.0

        # Live-центроїди кластерів (живі дрони) — оновлюються щокроку
        # у hybrid; для візуалізації хрестика. Сектори (заморожені) у
        # self.cluster_sectors уже встановлені вище.
        self.cluster_centroids = {}

        # Лічильник покриття для логу
        self._last_coverage_pct = 0

        # Метрики — скидання
        self.metrics.reset()
        self.metrics.mission_start_time = 0.0

        # Лог
        self.ui_panel.log_messages = []
        self._log("Симуляцію розпочато", 'info')
        self._log(f"Сценарій: {cfg.SCENARIO} | Seed: {cfg.MAP_SEED}", 'info')
        self._log(f"Дронів: {cfg.NUM_DRONES}, Цілей: {len(self.targets)}", 'info')

    # ========== ЛОГ ==========

    def _log(self, message, msg_type='info'):
        """Додати повідомлення до логу з поточною часовою міткою."""
        m = int(self.sim_time) // 60
        s = int(self.sim_time) % 60
        self.ui_panel.add_log(f"{m:02d}:{s:02d}", message, msg_type)

    # ========== ОСНОВНЕ ОНОВЛЕННЯ ==========

    def update(self, dt):
        """Повний крок симуляції."""
        if self.paused:
            return

        self.sim_time += dt

        # 1. Оновлення сусідів (комунікація)
        update_neighbors(self.drones)

        # 1b. Hybrid: переобрання лідерів кластерів і live-центроїди.
        # Самі кластери статичні — призначені у reset() і не змінюються.
        if cfg.SCENARIO == 'hybrid':
            self.cluster_centroids = compute_clusters(self.drones)
        else:
            self.cluster_centroids = {}

        # 2. Оновлення стану кожного дрона
        for drone in self.drones:
            was_active = drone.status != 'lost'
            drone.update(dt, self.ew_rect)
            # Виявлення втрати дрона
            if was_active and drone.status == 'lost' and not drone.loss_reported:
                self._handle_drone_loss(drone)

        # 3. Ройовий інтелект — вибір зон для розвідників.
        # Для hybrid передаємо сектор кластера → sector-бонус у swarm_scout.
        # Для leader_follower підлеглі летять у свій слот формації позаду
        # лідера — кожному свій порядковий номер серед живих followers.

        # 3a. Reroute (тільки hybrid): якщо risk_map обраної target_zone
        # зріс після вибору — скидаємо її, щоб swarm_scout перерахував і
        # дрон не летів у щойно виявлену РЕБ-зону.
        if cfg.SCENARIO == 'hybrid':
            for drone in self.drones:
                if drone.status != 'scouting' or drone.target_zone is None:
                    continue
                zx, zy = drone.target_zone
                if drone.risk_map[zy][zx] >= 0.6:
                    self._log(
                        f"[REROUTE] D{drone.id} скинув target_zone "
                        f"{drone.target_zone} (risk="
                        f"{drone.risk_map[zy][zx]:.2f})",
                        'threat')
                    drone.target_zone = None

        follower_idx = {}
        if cfg.SCENARIO == 'leader_follower':
            alive_followers = sorted(
                (d for d in self.drones
                 if d.status != 'lost' and not d.is_leader),
                key=lambda d: d.id)
            follower_idx = {d.id: i for i, d in enumerate(alive_followers)}

        for drone in self.drones:
            if drone.status == 'scouting' and drone.target_zone is None:
                sector = None
                leader = None
                slot = None
                if cfg.SCENARIO == 'hybrid':
                    sector = self.cluster_sectors.get(drone.cluster_id)
                elif (cfg.SCENARIO == 'leader_follower'
                        and not drone.is_leader):
                    # Єдиний глобальний лідер; кожен підлеглий — у свій
                    # слот формації. Лідер загинув — None, рій розбредається.
                    leader = next(
                        (d for d in self.drones
                         if d.is_leader and d.status != 'lost'),
                        None
                    )
                    slot = follower_idx.get(drone.id)
                swarm_scout(drone, self.drones, self.ew_rect,
                            sector_bounds=sector, follow_leader=leader,
                            follower_slot=slot)

        # 4. Рух дронів. Для swarm_only — справжній Boids (sep/ali/coh)
        # замість _move_to_zone; інші статуси (returning) йдуть звичайним
        # drone.move(). У всіх інших стратегіях — стандартний рух.
        for drone in self.drones:
            if (cfg.SCENARIO == 'swarm_only'
                    and drone.status == 'scouting'):
                boids_move(drone, self.drones, dt)
            else:
                drone.move(dt)

        # 4b. Камікадзе-втрати стаються під час move (крок 4), а не update
        # (крок 2), тож фіксуємо їх тут — лише метрика, без threat-логу й
        # РЕБ-поширення (це успішний удар, а не загроза).
        for drone in self.drones:
            if (drone.status == 'lost' and not drone.loss_reported
                    and drone.lost_reason == 'kamikaze'):
                drone.loss_reported = True
                self.metrics.record_drone_loss(
                    self.sim_time, drone, 'kamikaze')

        # 5. Виявлення цілей
        self._detect_targets()

        # 6. Оновлення цілей (таймер класифікації)
        for target in self.targets:
            old_state = target.state
            target.update(dt)
            if old_state == 'detected' and target.state == 'classified':
                self._log(
                    f"Ціль #{target.id} ({target.label}) класифіковано",
                    'warning')
                self.metrics.record_target_classified(self.sim_time, target)

        # 7. Призначення на класифіковані цілі
        if cfg.SCENARIO == 'greedy':
            # Кожен розвідник самостійно атакує найближчу відому ціль.
            # Дублювання дозволено — показує слабкість жадібного підходу.
            for drone in self.drones:
                if drone.status != 'scouting' or drone.battery < 15:
                    continue
                best_t, best_dist = None, float('inf')
                for tid in drone.known_targets:
                    target = next(
                        (t for t in self.targets if t.id == tid), None)
                    if target is None or target.state != 'classified':
                        continue
                    if target.auction_done:
                        # DEBUG: класифікована ціль пропущена
                        needed = (target.drones_needed
                                  - target.hits_received)
                        self._log(
                            f"[GREEDY-SKIP] D{drone.id}: пропущено "
                            f"{target.label} причина: "
                            f"auction_done={target.auction_done} / "
                            f"needed={needed} / battery={drone.battery:.0f}",
                            'warning')
                        continue
                    d = math.hypot(drone.x - target.x, drone.y - target.y)
                    if d < best_dist:
                        best_dist, best_t = d, target
                if best_t is not None:
                    drone.status = 'attacking'
                    drone.target_obj = best_t
                    drone.target_zone = None
        elif cfg.SCENARIO == 'swarm_only':
            pass  # дрони тільки розвідують, атак немає
        elif cfg.SCENARIO == 'leader_follower':
            run_leader_follower(self.drones, self.targets, self._log)
        elif cfg.SCENARIO == 'hybrid':
            # Дворівневий аукціон: L1 за сектором кластера, L2 глобальний.
            run_cluster_auction(
                self.drones, self.targets, self.risk_map,
                self.cluster_sectors, self._log,
                metrics_cb=lambda target, all_bids, winner_ids:
                    self.metrics.record_auction(
                        self.sim_time, target, all_bids, winner_ids))
        else:
            # 'auction' — плоский аукціон з bid (без кластерів і консенсусу)
            for target in self.targets:
                if target.state == 'classified' and not target.auction_done:
                    result = run_auction(
                        self.drones, target, self.risk_map, self._log)
                    if result and result['assigned']:
                        self.metrics.record_auction(
                            self.sim_time, target,
                            result['all_bids'], result['winner_ids'])

        # 8. Перевірка знищення цілей
        self._check_destruction()

        # 9. Дворівневий консенсус — тільки 'hybrid'.
        # L1 (intra-cluster) — часто, поширює покриття/цілі/ризик усередині.
        # L2 (inter-cluster) — втричі рідше, лідери діляться лише цілями.
        if cfg.SCENARIO == 'hybrid':
            self.consensus_timer += dt
            self.inter_consensus_timer += dt
            if self.consensus_timer >= cfg.CONSENSUS_INTERVAL:
                self.consensus_timer = 0.0
                intra_cluster_consensus(self.drones, self._log)
                self.metrics.record_consensus_round(self.sim_time)
            if self.inter_consensus_timer >= cfg.INTER_CLUSTER_INTERVAL:
                self.inter_consensus_timer = 0.0
                inter_cluster_consensus(self.drones, self._log)

        # 10. Оновлення глобального покриття
        self._update_coverage()

        # 11. Переназначення дронів при втратах
        self._check_reassignment()

        # 12. Збір метрик
        self.metrics.update(
            self.sim_time, dt, self.drones, self.global_coverage)

        # 13. Перевірка завершення місії
        self._check_mission_end()

    # ========== ВИЯВЛЕННЯ ЦІЛЕЙ ==========

    def _detect_targets(self):
        """Перевірка виявлення невідомих цілей дронами."""
        for target in self.targets:
            if target.state != 'undetected':
                continue
            for drone in self.drones:
                if drone.status == 'lost':
                    continue
                dist = math.hypot(drone.x - target.x, drone.y - target.y)
                if dist <= drone.detection_radius:
                    target.state = 'detected'
                    target.detect_timer = 0.0
                    drone.known_targets[target.id] = {
                        'type': target.type,
                        'x': target.x, 'y': target.y
                    }
                    self.last_progress_time = self.sim_time
                    self._log(
                        f"[D{drone.id}] виявлено ціль #{target.id}",
                        'warning')
                    self.metrics.record_target_detected(
                        self.sim_time, target)
                    break

    # ========== ЗНИЩЕННЯ ЦІЛЕЙ ==========

    def _check_destruction(self):
        """Лог і метрики для цілей, щойно переведених у 'destroyed'.

        Саме знищення виконується у drone._move_to_target (камікадзе).
        """
        for target in self.targets:
            if target.state != 'destroyed' or target.destruction_logged:
                continue
            target.destruction_logged = True
            self.last_progress_time = self.sim_time
            self._log(
                f"Ціль #{target.id} ({target.label}) знищено!",
                'auction')
            self.metrics.record_target_destroyed(self.sim_time, target)

    # ========== ВТРАТА ДРОНА ==========

    def _handle_drone_loss(self, drone):
        """Обробка втрати дрона: оновлення ризику, метрики, лог."""
        drone.loss_reported = True
        zone = drone.get_zone()

        # Визначення причини
        ex, ey, ew, eh = self.ew_rect
        is_ew = ex <= drone.x <= ex + ew and ey <= drone.y <= ey + eh

        # Метрики
        self.metrics.record_drone_loss(self.sim_time, drone, drone.lost_reason)

        # Оновлення карт ризику — лише для справжніх РЕБ-втрат.
        # РЕБ-адаптація увімкнена тільки у 'hybrid' (повна система):
        # 'auction' і 'greedy' не поширюють ризик між дронами,
        # 'leader_follower' демонструє вразливість ієрархії.
        if (cfg.SCENARIO == 'hybrid'
                and zone and is_ew and drone.lost_reason == 'ew'):
            col, row = zone

            # Накопичення впевненості: лічильник EW-втрат у зоні визначає
            # base_risk. 1 втрата = підозра (0.5), 2+ = підтверджено (0.9).
            self.loss_count_map[row][col] += 1
            loss_count = self.loss_count_map[row][col]
            base_risk = 0.9 if loss_count >= 2 else 0.5

            # КОМПОНЕНТ 2 — глобальна risk_map: зона загибелі = base_risk,
            # manhattan-сусіди слабше (×0.7 на dist 1, ×0.4 на dist 2).
            spread = {0: 1.0, 1: 0.7, 2: 0.4}
            for dr in range(-2, 3):
                for dc in range(-2, 3):
                    md = abs(dr) + abs(dc)
                    if md > 2:
                        continue
                    r, c = row + dr, col + dc
                    if 0 <= r < cfg.GRID_ROWS and 0 <= c < cfg.GRID_COLS:
                        self.risk_map[r][c] = max(
                            self.risk_map[r][c], base_risk * spread[md])

            # КОМПОНЕНТ 1 — миттєва реакція сусідів. Не чекаючи консенсус,
            # кожен живий дрон у межах comm_radius від точки загибелі
            # одразу оновлює власну risk_map: base_risk у зоні, ×0.66
            # навколо (radius 1). Інші дізнаються пізніше через консенсус.
            alerted = 0
            for other in self.drones:
                if other.status == 'lost':
                    continue
                dist = math.hypot(other.x - drone.x, other.y - drone.y)
                if dist > other.comm_radius:
                    continue
                other.risk_map[row][col] = max(
                    other.risk_map[row][col], base_risk)
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        r, c = row + dr, col + dc
                        if 0 <= r < cfg.GRID_ROWS and 0 <= c < cfg.GRID_COLS:
                            other.risk_map[r][c] = max(
                                other.risk_map[r][c], base_risk * 0.66)
                alerted += 1

            tag = 'REB-CONFIRMED' if loss_count >= 2 else 'REB-SUSPECT'
            self._log(
                f"[{tag}] зона {col},{row} "
                f"(втрат: {loss_count}, сповіщено: {alerted})",
                'threat')

        # Лог: хто виявив втрату
        reporter = None
        for other in self.drones:
            if other.status != 'lost' and drone in other.neighbors:
                reporter = other
                break

        if reporter and zone:
            self._log(
                f"[D{reporter.id}\u2192ALL] втрата D{drone.id} "
                f"в зоні {zone[0]},{zone[1]}",
                'threat')
        elif zone:
            self._log(f"Втрата D{drone.id} в зоні {zone[0]},{zone[1]}",
                      'threat')
        else:
            self._log(f"Втрата D{drone.id}", 'threat')

        # Лог про РЕБ
        if is_ew:
            self._log("[ALL] РЕБ загроза виявлена, режим обережності",
                      'threat')

    # ========== ПЕРЕНАЗНАЧЕННЯ ==========

    def _check_reassignment(self):
        """Якщо атакуючий дрон втрачено — скинути аукціон для цілі."""
        for target in self.targets:
            if target.state != 'classified' or not target.auction_done:
                continue
            active = sum(
                1 for d in self.drones
                if d.status == 'attacking' and d.target_obj is target
            )
            if active < target.drones_needed:
                target.auction_done = False
                target.assigned_drones = [
                    d.id for d in self.drones
                    if d.status == 'attacking' and d.target_obj is target
                ]

    # ========== ПОКРИТТЯ ==========

    def _update_coverage(self):
        """Оновлення глобальної карти покриття та метрик зон."""
        for drone in self.drones:
            if drone.status == 'lost':
                continue
            zone = drone.get_zone()
            if zone:
                col, row = zone
                self.global_coverage[row][col] = True
                self.metrics.record_zone_visit(row, col, drone.id)

        # Лог при досягненні нових порогів
        total = cfg.GRID_COLS * cfg.GRID_ROWS
        covered = sum(
            1 for r in range(cfg.GRID_ROWS)
            for c in range(cfg.GRID_COLS)
            if self.global_coverage[r][c]
        )
        pct = int(covered / total * 100)
        if pct >= self._last_coverage_pct + 25 and pct <= 100:
            self._last_coverage_pct = pct
            self._log(f"Покриття карти: {pct}%", 'info')

    # ========== ЗАВЕРШЕННЯ МІСІЇ ==========

    def _check_mission_end(self):
        """Три умови завершення місії."""
        if self.mission_complete or self.mission_failed:
            if not self._report_generated:
                self._report_generated = True
                self._save_and_report()
            return

        # 1. Всі цілі знищено
        if all(t.state == 'destroyed' for t in self.targets):
            self.mission_complete = True
            self._end_reason = 'success'
            self._log("Всі цілі знищено! Місію виконано!", 'auction')

        # 2. Всі дрони втрачено
        elif all(d.status == 'lost' for d in self.drones):
            self.mission_failed = True
            self._end_reason = 'all_lost'
            self._log("Всі дрони втрачено! Місію провалено!", 'threat')

        # 3. Застій — немає прогресу понад STAGNATION_TIME
        elif (not self.stagnation_triggered
              and self.sim_time - self.last_progress_time
              > cfg.STAGNATION_TIME):
            self.stagnation_triggered = True
            self.mission_complete = True
            self._end_reason = 'stagnation'
            self._log(
                f"Район зачищено (застій {cfg.STAGNATION_TIME:.0f}с). "
                "Місію завершено!", 'auction')

    def _save_and_report(self):
        """Зберегти CSV, вивести фінальний рядок у лог і згенерувати звіт."""
        self.metrics.mission_end_time = self.sim_time
        destroyed = sum(1 for t in self.targets if t.state == 'destroyed')
        total = len(self.targets)
        pct = int(destroyed / total * 100) if total else 0
        reason_label = {
            'success':    'всі цілі знищено',
            'all_lost':   'всі дрони втрачено',
            'stagnation': 'застій',
        }.get(getattr(self, '_end_reason', ''), 'невідомо')
        m = int(self.sim_time) // 60
        s = int(self.sim_time) % 60
        self._log(
            f"Підсумок: {destroyed}/{total} цілей ({pct}%) | "
            f"Час: {m:02d}:{s:02d} | Причина: {reason_label}",
            'info')
        directory = self.metrics.export_to_csv()
        self._log(f"Дані збережено в {directory}/", 'info')
        self.metrics.generate_report(directory)

    # ========== МАЛЮВАННЯ ==========

    def draw(self, screen):
        """Повне малювання сцени."""
        # Активні зони (де зараз є дрони)
        active_zones = set()
        for drone in self.drones:
            if drone.status != 'lost':
                zone = drone.get_zone()
                if zone:
                    active_zones.add(zone)

        # 1. Карта з зонами, теплокартою ризику та РЕБ
        self.map_renderer.draw(screen, self.global_coverage,
                               active_zones, self.ew_rect, self.risk_map)

        # 2. Mesh з'єднання між дронами — тільки hybrid
        if self.current_strategy == 'hybrid':
            draw_hybrid_mesh(screen, self.drones)

        # 2b. Сектори + центроїди живих дронів кластера (hybrid)
        if self.current_strategy == 'hybrid' and self.cluster_sectors:
            _pal = [(80, 160, 255), (80, 255, 160), (255, 200, 80),
                    (255, 80, 160), (200, 120, 255)]
            sect_surf = pygame.Surface(
                (cfg.MAP_WIDTH, cfg.WINDOW_HEIGHT), pygame.SRCALPHA)
            # Напівпрозоре заливання кожного сектора (alpha ~22)
            for cid in sorted(self.cluster_sectors):
                cc = _pal[cid % len(_pal)]
                x_start, x_end = self.cluster_sectors[cid]
                rect = pygame.Rect(
                    int(x_start), 0,
                    int(x_end - x_start), cfg.WINDOW_HEIGHT)
                pygame.draw.rect(sect_surf, (*cc, 22), rect)
            # Тонкі вертикальні лінії на правих межах секторів (alpha ~80)
            for cid in sorted(self.cluster_sectors):
                if cid >= len(self.cluster_sectors) - 1:
                    continue  # права межа карти — лінію не малюємо
                cc = _pal[cid % len(_pal)]
                line_x = int(self.cluster_sectors[cid][1])
                pygame.draw.line(
                    sect_surf, (*cc, 80),
                    (line_x, 0), (line_x, cfg.WINDOW_HEIGHT), 2)
            screen.blit(sect_surf, (0, 0))
            # Хрестики live-центроїдів кластерів
            for cid, (cx, cy) in self.cluster_centroids.items():
                cc = _pal[cid % len(_pal)]
                ix, iy = int(cx), int(cy)
                pygame.draw.line(screen, cc, (ix - 8, iy), (ix + 8, iy), 2)
                pygame.draw.line(screen, cc, (ix, iy - 8), (ix, iy + 8), 2)
                pygame.draw.circle(screen, cc, (ix, iy), 3)

        # 3. Лінії атаки від дронів до цілей
        for drone in self.drones:
            drone.draw_attack_line(screen)

        # 4. Цілі
        for target in self.targets:
            target.draw(screen, self.fonts)

        # 5. Дрони
        for drone in self.drones:
            drone.draw(screen, self.fonts)

        # 6. Права панель
        self.ui_panel.draw(
            screen, self.sim_time, self.drones, self.targets,
            self.global_coverage, self.selected_drone, self.selected_target)

        # 7. Підказки
        self._draw_hints(screen)

    def _draw_hints(self, screen):
        """Підказки керування та стан паузи."""
        if self.paused:
            txt = self.font_pause.render('[ПАУЗА]', True, (255, 240, 80))
            screen.blit(txt, (cfg.MAP_WIDTH // 2 - txt.get_width() // 2,
                              cfg.WINDOW_HEIGHT // 2 - 15))

        # Підказки знизу
        hints = ("Space: пауза | R: рестарт | S: звіт | "
                 "1-5: стратегія | +/-: швидкість | Esc: вихід")
        h_surf = self.font_hint.render(hints, True, (100, 100, 110))
        screen.blit(h_surf, (8, cfg.WINDOW_HEIGHT - 18))

        # Поточна стратегія + множник швидкості
        speed_txt = (f"  |  Швидкість: {self.speed_multiplier}x"
                     if self.speed_multiplier > 1 else "")
        strat_txt = f"Стратегія: {self.current_strategy}{speed_txt}"
        s_surf = self.font_hint.render(strat_txt, True, (140, 200, 140))
        screen.blit(s_surf, (8, cfg.WINDOW_HEIGHT - 34))

    # ========== ПОДІЇ ==========

    def handle_event(self, event):
        """Обробка подій клавіатури та миші."""
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                self.paused = not self.paused
                st = "призупинено" if self.paused else "продовжено"
                self._log(f"Симуляцію {st}", 'info')

            elif event.key == pygame.K_r:
                self._new_game()

            elif event.key == pygame.K_s:
                self._log("Збереження даних та генерація звіту...", 'info')
                self._save_and_report()

            elif event.key == pygame.K_1:
                self._switch_strategy('greedy')

            elif event.key == pygame.K_2:
                self._switch_strategy('swarm_only')

            elif event.key == pygame.K_3:
                self._switch_strategy('auction')

            elif event.key == pygame.K_4:
                self._switch_strategy('hybrid')

            elif event.key == pygame.K_5:
                self._switch_strategy('leader_follower')

            elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                steps = [1, 2, 3, 5]
                idx = steps.index(self.speed_multiplier) if self.speed_multiplier in steps else 0
                self.speed_multiplier = steps[(idx + 1) % len(steps)]

            elif event.key == pygame.K_MINUS:
                steps = [1, 2, 3, 5]
                idx = steps.index(self.speed_multiplier) if self.speed_multiplier in steps else 0
                self.speed_multiplier = steps[(idx - 1) % len(steps)]

            elif event.key == pygame.K_ESCAPE:
                pygame.quit()
                sys.exit()

        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos
            if mx < cfg.MAP_WIDTH:
                self._handle_click(mx, my)

    def _switch_strategy(self, name):
        """Перемкнути стратегію та перезапустити з тими ж умовами (MAP_SEED)."""
        if name == self.current_strategy:
            return
        self.current_strategy = name
        # Синхронізуємо cfg.SCENARIO, щоб гілки в update() (аукціон,
        # консенсус, РЕБ-адаптація, лідер-послідовувач) перемикались.
        cfg.SCENARIO = name
        # reset() використовує cfg.MAP_SEED і зберігає self.current_strategy,
        # тож стартові умови ідентичні для всіх стратегій
        self.reset()
        self._log(f"Стратегія: {name}", 'info')

    def _handle_click(self, mx, my):
        """Клік по карті — виділення дрона або цілі."""
        # Зняти попереднє виділення
        if self.selected_drone is not None:
            self.selected_drone.selected = False
            self.selected_drone = None
        if self.selected_target is not None:
            self.selected_target.selected = False
            self.selected_target = None

        # Пошук дрона
        for drone in self.drones:
            if math.hypot(drone.x - mx, drone.y - my) < 16:
                drone.selected = True
                self.selected_drone = drone
                return

        # Пошук цілі
        for target in self.targets:
            if target.state == 'undetected':
                continue
            if math.hypot(target.x - mx, target.y - my) < 16:
                target.selected = True
                self.selected_target = target
                return


# ==================== ТОЧКА ВХОДУ ====================

def main():
    """Запуск симуляції."""
    pygame.init()

    screen = pygame.display.set_mode((cfg.WINDOW_WIDTH, cfg.WINDOW_HEIGHT))
    pygame.display.set_caption(
        'Симуляція рою БПЛА \u2014 Координація з децентралізованими стратегіями')
    clock = pygame.time.Clock()

    sim = Simulation()

    while True:
        dt = clock.tick(cfg.FPS) / 1000.0
        dt = min(dt, 0.05)   # захист від стрибків при затримках

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            sim.handle_event(event)

        for _ in range(sim.speed_multiplier):
            sim.update(dt)

        screen.fill(cfg.COLOR_BG)
        sim.draw(screen)
        pygame.display.flip()


if __name__ == '__main__':
    main()
