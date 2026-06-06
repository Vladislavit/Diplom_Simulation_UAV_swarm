# drone.py — Клас Drone: стан, рух, батарея, малювання FPV-дрона

import pygame
import math
import random
import config as cfg


class Drone:
    """FPV-дрон з позицією, статусом, батареєю та комунікацією."""

    def __init__(self, drone_id, x, y):
        self.id = drone_id
        self.x = float(x)
        self.y = float(y)
        self.vx = 0.0
        self.vy = 0.0
        self.speed = cfg.DRONE_SPEED
        self.battery = 100.0
        self.status = 'scouting'  # scouting / attacking / returning / lost

        # Радіуси
        self.detection_radius = cfg.DETECTION_RADIUS
        self.comm_radius = cfg.COMM_RADIUS

        # Навігація
        self.target_zone = None   # (col, row) — цільова зона розвідки
        self.target_obj = None    # об'єкт Target для атаки
        self.bid = 0.0            # значення bid після аукціону

        # Комунікація та карти
        self.neighbors = []       # список сусідніх дронів у радіусі зв'язку
        self.local_map = [[False] * cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        self.known_targets = {}   # target_id -> dict з інформацією
        self.risk_map = [[0.0] * cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        # Стигмергія (swarm_only): цифрові феромони відвіданих зон
        self.pheromone_map = [[0.0] * cfg.GRID_COLS
                              for _ in range(cfg.GRID_ROWS)]

        # РЕБ
        self.ew_timer = 0.0
        self.ew_threshold = random.uniform(cfg.EW_LOSS_TIME_MIN, cfg.EW_LOSS_TIME_MAX)
        self.loss_reported = False

        # База (для повернення)
        self.home_x = float(x)
        self.home_y = float(y)

        # Інтерфейс
        self.selected = False

        # Патрулювання / діагностика втрати
        self.scout_idle_time = 0.0
        self.lost_reason = None

        # Лідер у стратегії leader_follower / лідер кластера (оновлюється щокроку)
        self.is_leader = False

        # Кластер у стратегії hybrid (-1 = поза кластером)
        self.cluster_id = -1

    # ========== ОНОВЛЕННЯ СТАНУ ==========

    def update(self, dt, ew_rect):
        """Оновлення стану: батарея, РЕБ, покриття зон."""
        if self.status == 'lost':
            return

        # Дрон на базі — мінімальний розряд
        if self.status == 'returning':
            dist_home = math.hypot(self.home_x - self.x, self.home_y - self.y)
            if dist_home <= 15:
                self.vx = 0.0
                self.vy = 0.0
                self.battery = max(0, self.battery - cfg.BATTERY_DRAIN_RATE * 0.05 * dt)
                return

        # Розряд батареї
        self.battery = max(0.0, self.battery - cfg.BATTERY_DRAIN_RATE * dt)
        if self.battery <= 0:
            self.status = 'lost'
            self.lost_reason = 'battery'
            return

        # Перевірка низького заряду — лише для розвідників
        if self.battery < cfg.LOW_BATTERY and self.status == 'scouting':
            self.status = 'returning'
            self.target_zone = None
            self.target_obj = None

        # Перевірка РЕБ зони
        ex, ey, ew, eh = ew_rect
        if ex <= self.x <= ex + ew and ey <= self.y <= ey + eh:
            self.ew_timer += dt
            if self.ew_timer >= self.ew_threshold:
                self.status = 'lost'
                self.lost_reason = 'ew'
                return
        else:
            self.ew_timer = max(0.0, self.ew_timer - dt * 0.3)

        # Оновлення покриття локальної карти
        zone = self.get_zone()
        if zone:
            col, row = zone
            self.local_map[row][col] = True

        # Таймер простою розвідки (для форсованого вибору квадранта)
        if self.status == 'scouting' and self.target_zone is None:
            self.scout_idle_time += dt
        else:
            self.scout_idle_time = 0.0

    # ========== РУХ ==========

    def move(self, dt):
        """Застосування руху залежно від статусу."""
        if self.status == 'lost':
            return

        if self.status == 'attacking':
            self._move_to_target(dt)
        elif self.status == 'returning':
            self._move_to_home(dt)
        elif self.status == 'scouting':
            self._move_to_zone(dt)

        # Обмеження позиції в межах карти
        self.x = max(12, min(cfg.MAP_WIDTH - 12, self.x))
        self.y = max(12, min(cfg.WINDOW_HEIGHT - 12, self.y))

    def _move_to_zone(self, dt):
        """Рух до цільової зони розвідки з інерцією."""
        if self.target_zone is None:
            return
        col, row = self.target_zone
        tx = col * cfg.ZONE_W + cfg.ZONE_W // 2
        ty = row * cfg.ZONE_H + cfg.ZONE_H // 2
        self._steer_toward(tx, ty, self.speed, dt)

        # Якщо досягли центру зони — зняти ціль
        if math.hypot(tx - self.x, ty - self.y) < 15:
            self.target_zone = None

    def _move_to_target(self, dt):
        """Камікадзе до цілі: на контакті — удар, дрон витрачається."""
        if self.target_obj is None or self.target_obj.state == 'destroyed':
            self.status = 'scouting'
            self.target_obj = None
            self.target_zone = None
            return

        target = self.target_obj
        self._steer_toward(target.x, target.y, self.speed * 1.2, dt)

        # Контакт — один камікадзе-удар (+1 до hits_received, без шансу)
        if math.hypot(target.x - self.x, target.y - self.y) <= cfg.DESTROY_DISTANCE:
            target.hits_received += 1
            if self.id in target.assigned_drones:
                target.assigned_drones.remove(self.id)
            self.status = 'lost'
            self.lost_reason = 'kamikaze'
            self.target_obj = None
            # Знищення при досягненні фіксованої к-сті ударів
            if target.hits_received >= target.drones_needed:
                target.state = 'destroyed'

    def _move_to_home(self, dt):
        """Повернення на базу."""
        self._steer_toward(self.home_x, self.home_y, self.speed, dt)

    def _steer_toward(self, tx, ty, speed, dt):
        """Плавний рух до точки (tx, ty) з інерцією."""
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)
        if dist < 2:
            self.vx *= 0.5
            self.vy *= 0.5
            return
        nx = dx / dist
        ny = dy / dist
        self.vx = self.vx * cfg.INERTIA + nx * speed * (1 - cfg.INERTIA)
        self.vy = self.vy * cfg.INERTIA + ny * speed * (1 - cfg.INERTIA)
        self.x += self.vx * dt
        self.y += self.vy * dt

    # ========== УТИЛІТИ ==========

    def get_zone(self):
        """Повертає (col, row) поточної зони або None."""
        col = int(self.x // cfg.ZONE_W)
        row = int(self.y // cfg.ZONE_H)
        if 0 <= col < cfg.GRID_COLS and 0 <= row < cfg.GRID_ROWS:
            return (col, row)
        return None

    def get_color(self):
        """Колір дрона залежно від статусу та заряду."""
        if self.status == 'lost':
            return cfg.COLOR_DRONE_LOST
        if self.battery < cfg.LOW_BATTERY:
            return cfg.COLOR_DRONE_LOW_BAT
        if self.status == 'attacking':
            return cfg.COLOR_DRONE_ATTACK
        return cfg.COLOR_DRONE_SCOUT

    def get_status_text(self):
        """Статус українською."""
        return {
            'scouting': 'Розвідка',
            'attacking': 'Атака',
            'returning': 'Повернення',
            'lost': 'Втрачено',
        }.get(self.status, self.status)

    # ========== МАЛЮВАННЯ ==========

    def draw(self, surface, fonts):
        """Малювання дрона на карті."""
        x, y = int(self.x), int(self.y)
        color = self.get_color()

        if self.status == 'lost':
            self._draw_lost(surface, x, y, color)
            return

        # Хрестик — 4 промені
        size = 9
        pygame.draw.line(surface, color, (x - size, y), (x + size, y), 2)
        pygame.draw.line(surface, color, (x, y - size), (x, y + size), 2)
        # Коло по центру
        pygame.draw.circle(surface, color, (x, y), 5)
        pygame.draw.circle(surface, (0, 0, 0), (x, y), 5, 1)

        # Лідер (leader_follower) — постійне жовте коло навколо
        if self.is_leader:
            pygame.draw.circle(surface, (255, 215, 40), (x, y), 13, 2)

        # Кластер (hybrid) — кольоровий маркер зі зміщенням (+8, -8).
        # Follower — квадрат 3×3; лідер кластера — ромб того ж кольору.
        if cfg.SCENARIO == 'hybrid' and self.cluster_id >= 0:
            palette = [(80, 160, 255), (80, 255, 160),
                       (255, 200, 80), (255, 80, 160)]
            cc = palette[self.cluster_id % 4]
            mx, my = x + 8, y - 8
            if self.is_leader:
                pts = [(mx, my - 4), (mx + 4, my),
                       (mx, my + 4), (mx - 4, my)]
                pygame.draw.polygon(surface, cc, pts)
            else:
                pygame.draw.rect(surface, cc, (mx - 1, my - 1, 3, 3))

        # Радіус виявлення — пунктирне коло (колір кластера тільки в hybrid)
        if cfg.SCENARIO == 'hybrid' and self.cluster_id >= 0:
            _det_palette = {0: (100, 180, 255), 1: (100, 255, 160),
                            2: (255, 210, 80),  3: (255, 120, 80),
                            4: (200, 100, 255)}
            det_color = _det_palette.get(self.cluster_id, color)
        else:
            det_color = color
        _draw_dashed_circle(surface, x, y, self.detection_radius, det_color, 1)

        # Ідентифікатор
        label = fonts['tiny'].render(f'D{self.id}', True, color)
        surface.blit(label, (x + 11, y - 14))

        # Виділення
        if self.selected:
            pygame.draw.circle(surface, (255, 255, 255), (x, y), 16, 2)

    def _draw_lost(self, surface, x, y, color):
        """Напівпрозорий пунктирний дрон — статус lost."""
        s = pygame.Surface((44, 44), pygame.SRCALPHA)
        cx, cy = 22, 22
        ac = (color[0], color[1], color[2], 80)
        # Пунктирний хрестик
        for i in range(0, 10, 4):
            pygame.draw.line(s, ac, (cx - 9 + i, cy), (cx - 7 + i, cy), 2)
            pygame.draw.line(s, ac, (cx + 1 + i, cy), (cx + 3 + i, cy), 2)
            pygame.draw.line(s, ac, (cx, cy - 9 + i), (cx, cy - 7 + i), 2)
            pygame.draw.line(s, ac, (cx, cy + 1 + i), (cx, cy + 3 + i), 2)
        # Пунктирне коло
        for deg in range(0, 360, 30):
            a = math.radians(deg)
            a2 = math.radians(deg + 15)
            p1 = (cx + int(5 * math.cos(a)), cy + int(5 * math.sin(a)))
            p2 = (cx + int(5 * math.cos(a2)), cy + int(5 * math.sin(a2)))
            pygame.draw.line(s, ac, p1, p2, 1)
        surface.blit(s, (x - 22, y - 22))

    def draw_attack_line(self, surface):
        """Помаранчева пунктирна лінія зі стрілкою від дрона до цілі."""
        if self.status != 'attacking' or self.target_obj is None:
            return
        if self.target_obj.state == 'destroyed':
            return
        start = (int(self.x), int(self.y))
        end = (int(self.target_obj.x), int(self.target_obj.y))
        color = cfg.COLOR_DRONE_ATTACK
        _draw_dashed_line(surface, color, start, end, width=1, dash=8, gap=5)
        _draw_arrowhead(surface, color, start, end, size=8)


# ========== ДОПОМІЖНІ ФУНКЦІЇ МАЛЮВАННЯ ==========

def _draw_dashed_circle(surface, cx, cy, radius, color, width=1):
    """Пунктирне коло — чергування сегментів."""
    segments = 40
    for i in range(0, segments, 2):
        a1 = 2 * math.pi * i / segments
        a2 = 2 * math.pi * (i + 1) / segments
        x1 = cx + int(radius * math.cos(a1))
        y1 = cy + int(radius * math.sin(a1))
        x2 = cx + int(radius * math.cos(a2))
        y2 = cy + int(radius * math.sin(a2))
        pygame.draw.line(surface, color, (x1, y1), (x2, y2), width)


def _draw_dashed_line(surface, color, start, end, width=1, dash=8, gap=5):
    """Пунктирна лінія між двома точками."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1:
        return
    nx = dx / length
    ny = dy / length
    pos = 0.0
    while pos < length:
        sx = int(start[0] + nx * pos)
        sy = int(start[1] + ny * pos)
        epos = min(pos + dash, length)
        ex = int(start[0] + nx * epos)
        ey = int(start[1] + ny * epos)
        pygame.draw.line(surface, color, (sx, sy), (ex, ey), width)
        pos += dash + gap


def _draw_arrowhead(surface, color, start, end, size=8):
    """Стрілка на кінці лінії."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1:
        return
    angle = math.atan2(dy, dx)
    spread = math.pi / 6   # 30 градусів
    lx = end[0] - size * math.cos(angle - spread)
    ly = end[1] - size * math.sin(angle - spread)
    rx = end[0] - size * math.cos(angle + spread)
    ry = end[1] - size * math.sin(angle + spread)
    pygame.draw.line(surface, color, end, (int(lx), int(ly)), 2)
    pygame.draw.line(surface, color, end, (int(rx), int(ry)), 2)


_CLUSTER_PALETTE = [
    (80, 160, 255), (80, 255, 160), (255, 200, 80),
    (255, 80, 160), (200, 120, 255),
]


def draw_hybrid_mesh(surface, drones):
    """Двохрівнева візуалізація зв'язків для стратегії hybrid.

    Рівень 1 — внутрішньокластерні: суцільна лінія 1px у кольорі кластера,
    між живими дронами одного cluster_id у межах comm_radius.
    Рівень 2 — міжкластерні: пунктир 2px золотий, тільки між лідерами
    (is_leader=True) у межах comm_radius.
    """
    # Рівень 1: внутрішньокластерні суцільні лінії
    drawn = set()
    for d in drones:
        if d.status == 'lost' or d.cluster_id < 0:
            continue
        color = _CLUSTER_PALETTE[d.cluster_id % len(_CLUSTER_PALETTE)]
        for n in d.neighbors:
            if n.status == 'lost' or n.cluster_id != d.cluster_id:
                continue
            pair = (min(d.id, n.id), max(d.id, n.id))
            if pair in drawn:
                continue
            drawn.add(pair)
            pygame.draw.line(surface, color,
                             (int(d.x), int(d.y)),
                             (int(n.x), int(n.y)), 1)

    # Рівень 2: міжкластерні пунктири між лідерами
    leaders = [d for d in drones if d.is_leader and d.status != 'lost']
    drawn_leaders = set()
    for i, la in enumerate(leaders):
        for lb in leaders[i + 1:]:
            dist = math.hypot(la.x - lb.x, la.y - lb.y)
            if dist > la.comm_radius:
                continue
            pair = (min(la.id, lb.id), max(la.id, lb.id))
            if pair in drawn_leaders:
                continue
            drawn_leaders.add(pair)
            _draw_dashed_line(surface, (255, 215, 0),
                              (int(la.x), int(la.y)),
                              (int(lb.x), int(lb.y)),
                              width=2, dash=8, gap=5)
