# map_renderer.py — Процедурна генерація карти, зони покриття, РЕБ зона

import pygame
import math
import numpy as np
import config as cfg


class MapRenderer:
    """Генерація ландшафту та малювання карти з шарами."""

    def __init__(self):
        self.terrain = self._generate_terrain()
        self.grid_overlay = self._create_grid_overlay()

        # Поверхні зон
        self.zone_covered = pygame.Surface(
            (cfg.ZONE_W, cfg.ZONE_H), pygame.SRCALPHA)
        self.zone_covered.fill(cfg.COLOR_ZONE_COVERED)

        self.zone_active = pygame.Surface(
            (cfg.ZONE_W, cfg.ZONE_H), pygame.SRCALPHA)
        self.zone_active.fill(cfg.COLOR_ZONE_ACTIVE)

        # РЕБ зона (розмір фіксований; позиція передається у draw())
        self.ew_surface = pygame.Surface((cfg.EW_W, cfg.EW_H), pygame.SRCALPHA)
        self.ew_surface.fill(cfg.COLOR_EW_FILL)

        # Шрифти
        self.font_ew = pygame.font.SysFont('arial', 16, bold=True)

    # ========== ГЕНЕРАЦІЯ ЛАНДШАФТУ ==========

    def _generate_terrain(self):
        """Процедурна карта: поля, ліси, дороги, річка, будівлі."""
        surface = pygame.Surface((cfg.MAP_WIDTH, cfg.WINDOW_HEIGHT))
        rng = np.random.RandomState(cfg.MAP_SEED)

        # 1. Фон — темно-зелені поля
        surface.fill(cfg.COLOR_FIELD)

        # Текстура полів — випадкові плями
        for _ in range(350):
            px = rng.randint(0, cfg.MAP_WIDTH)
            py = rng.randint(0, cfg.WINDOW_HEIGHT)
            shade = rng.randint(-10, 10)
            c = tuple(
                max(0, min(255, cfg.COLOR_FIELD[i] + shade)) for i in range(3)
            )
            r = rng.randint(4, 14)
            pygame.draw.circle(surface, c, (int(px), int(py)), int(r))

        # 2. Ліси — темніші овальні зони
        forests = [
            (110, 70, 140, 95),   (360, 140, 110, 75),
            (710, 80, 150, 85),   (75, 340, 120, 90),
            (460, 440, 100, 80),  (810, 390, 130, 95),
            (240, 540, 105, 70),  (610, 570, 135, 80),
            (160, 190, 85, 65),   (760, 250, 95, 70),
        ]
        for fx, fy, fw, fh in forests:
            fx += int(rng.randint(-20, 20))
            fy += int(rng.randint(-20, 20))
            fs = pygame.Surface((fw, fh), pygame.SRCALPHA)
            pygame.draw.ellipse(fs, (*cfg.COLOR_FOREST, 200), (0, 0, fw, fh))
            # Деталі лісу
            for _ in range(25):
                tx = int(rng.randint(6, fw - 6))
                ty = int(rng.randint(6, fh - 6))
                sh = int(rng.randint(-6, 6))
                tc = tuple(
                    max(0, min(255, cfg.COLOR_FOREST[i] + sh)) for i in range(3)
                )
                tr = int(rng.randint(3, 9))
                pygame.draw.circle(fs, (*tc, 170), (tx, ty), tr)
            surface.blit(fs, (fx - fw // 2, fy - fh // 2))

        # 3. Дороги
        road_y = cfg.WINDOW_HEIGHT // 3       # горизонтальна (~240)
        road_x = cfg.MAP_WIDTH * 2 // 5       # вертикальна (~376)
        road_w = 14

        # Горизонтальна
        pygame.draw.rect(surface, cfg.COLOR_ROAD,
                         (0, road_y - road_w // 2, cfg.MAP_WIDTH, road_w))
        for rx in range(0, cfg.MAP_WIDTH, 30):
            pygame.draw.rect(surface, (200, 195, 175), (rx, road_y - 1, 15, 2))

        # Вертикальна
        pygame.draw.rect(surface, cfg.COLOR_ROAD,
                         (road_x - road_w // 2, 0, road_w, cfg.WINDOW_HEIGHT))
        for ry in range(0, cfg.WINDOW_HEIGHT, 30):
            pygame.draw.rect(surface, (200, 195, 175), (road_x - 1, ry, 2, 15))

        # 4. Річка — звивиста синя смуга
        river_pts = []
        rx = 220.0
        for ry in range(0, cfg.WINDOW_HEIGHT + 5, 4):
            rx += math.sin(ry * 0.012) * 1.6 + math.sin(ry * 0.028) * 0.9
            rx += float(rng.uniform(-0.5, 0.5))
            rx = max(60, min(cfg.MAP_WIDTH - 60, rx))
            river_pts.append((int(rx), ry))

        if len(river_pts) > 1:
            # Тінь річки
            pygame.draw.lines(surface, (35, 70, 130), False, river_pts, 18)
            # Основне русло
            pygame.draw.lines(surface, cfg.COLOR_RIVER, False, river_pts, 11)
            # Блики
            for i in range(0, len(river_pts) - 1, 10):
                pygame.draw.line(surface, (80, 140, 215),
                                 river_pts[i], river_pts[i + 1], 3)

        # 5. Будівлі — кластери сірих прямокутників
        clusters = [
            (road_x - 55, road_y - 75, 5),
            (road_x + 55, road_y + 35, 4),
            (road_x - 45, road_y + 95, 3),
            (660, 135, 4),
            (510, 490, 3),
            (815, 540, 4),
        ]
        for bcx, bcy, count in clusters:
            bcx += int(rng.randint(-15, 15))
            bcy += int(rng.randint(-12, 12))
            for _ in range(count):
                bx = bcx + int(rng.randint(-38, 38))
                by = bcy + int(rng.randint(-32, 32))
                bw = int(rng.randint(14, 30))
                bh = int(rng.randint(12, 24))
                pygame.draw.rect(surface, cfg.COLOR_BUILDING,
                                 (bx, by, bw, bh))
                pygame.draw.rect(surface, cfg.COLOR_BUILDING_EDGE,
                                 (bx, by, bw, bh), 1)

        return surface

    def _create_grid_overlay(self):
        """Статична сітка зон — тонкі напівпрозорі лінії."""
        surf = pygame.Surface(
            (cfg.MAP_WIDTH, cfg.WINDOW_HEIGHT), pygame.SRCALPHA)
        gc = cfg.COLOR_ZONE_GRID
        for col in range(cfg.GRID_COLS + 1):
            x = col * cfg.ZONE_W
            pygame.draw.line(surf, gc, (x, 0), (x, cfg.WINDOW_HEIGHT), 1)
        for row in range(cfg.GRID_ROWS + 1):
            y = row * cfg.ZONE_H
            pygame.draw.line(surf, gc, (0, y), (cfg.MAP_WIDTH, y), 1)
        return surf

    # ========== МАЛЮВАННЯ ==========

    def draw(self, screen, coverage, active_zones, ew_rect, risk_map=None):
        """Повне малювання карти: ландшафт → зони → теплокарта → РЕБ."""
        # Фон ландшафту
        screen.blit(self.terrain, (0, 0))

        # Зони покриття
        for row in range(cfg.GRID_ROWS):
            for col in range(cfg.GRID_COLS):
                zx = col * cfg.ZONE_W
                zy = row * cfg.ZONE_H
                if (col, row) in active_zones:
                    screen.blit(self.zone_active, (zx, zy))
                elif coverage[row][col]:
                    screen.blit(self.zone_covered, (zx, zy))

        # Сітка
        screen.blit(self.grid_overlay, (0, 0))

        # Теплова карта ризику — як рій "пізнає" РЕБ (тільки hybrid)
        if cfg.SCENARIO == 'hybrid' and risk_map is not None:
            self._draw_risk_heatmap(screen, risk_map)

        # РЕБ зона
        self._draw_ew_zone(screen, ew_rect)

    def _draw_risk_heatmap(self, screen, risk_map):
        """Напівпрозорі червоні зони за рівнем risk_map (> 0.3).

        alpha пропорційний ризику: int(risk * 100). Візуалізує, як рій
        поступово картографує небезпечну РЕБ-зону через втрати дронів.
        """
        heat = pygame.Surface(
            (cfg.MAP_WIDTH, cfg.WINDOW_HEIGHT), pygame.SRCALPHA)
        for row in range(cfg.GRID_ROWS):
            for col in range(cfg.GRID_COLS):
                risk = risk_map[row][col]
                if risk <= 0.3:
                    continue
                alpha = int(risk * 100)
                heat.fill(
                    (220, 50, 50, alpha),
                    (col * cfg.ZONE_W, row * cfg.ZONE_H,
                     cfg.ZONE_W, cfg.ZONE_H))
        screen.blit(heat, (0, 0))

    def _draw_ew_zone(self, screen, ew_rect):
        """РЕБ зона: напівпрозора заливка, пунктирна границя, підпис."""
        ex, ey, ew, eh = ew_rect

        # Заливка
        screen.blit(self.ew_surface, (ex, ey))

        # Пунктирна границя
        bc = cfg.COLOR_EW_BORDER
        d, g = 10, 6   # dash, gap

        # Горизонтальні
        for x in range(ex, ex + ew, d + g):
            x2 = min(x + d, ex + ew)
            pygame.draw.line(screen, bc, (x, ey), (x2, ey), 2)
            pygame.draw.line(screen, bc, (x, ey + eh), (x2, ey + eh), 2)

        # Вертикальні
        for y in range(ey, ey + eh, d + g):
            y2 = min(y + d, ey + eh)
            pygame.draw.line(screen, bc, (ex, y), (ex, y2), 2)
            pygame.draw.line(screen, bc, (ex + ew, y), (ex + ew, y2), 2)

        # Підпис
        label = self.font_ew.render('РЕБ', True, (255, 80, 80))
        lx = ex + ew // 2 - label.get_width() // 2
        ly = ey + 8
        screen.blit(label, (lx, ly))
