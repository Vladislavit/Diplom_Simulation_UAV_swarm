# targets.py — Клас Target: типи цілей, стани, візуалізація на карті

import pygame
import math
import config as cfg


class Target:
    """Ціль на карті — піхота, авто, БМП або танк."""

    def __init__(self, target_id, x, y, target_type):
        self.id = target_id
        self.x = x
        self.y = y
        self.type = target_type
        self.state = 'undetected'  # undetected / detected / classified / destroyed
        self.info = cfg.TARGET_TYPES[target_type]
        # Фіксована к-сть ударів для знищення (за типом цілі)
        self.drones_needed = cfg.DRONES_NEEDED[target_type]
        self.label = self.info['label']
        self.detect_timer = 0.0       # лічильник часу для класифікації
        self.assigned_drones = []     # id призначених дронів
        self.pulse_phase = 0.0        # фаза пульсації
        self.selected = False         # чи виділена користувачем
        self.auction_done = False     # чи вже проведено аукціон
        self.hits_received = 0        # кількість камікадзе-ударів
        self.destruction_logged = False  # чи задокументовано знищення

    def update(self, dt):
        """Оновлення стану цілі — таймер класифікації, пульсація."""
        self.pulse_phase += dt * 3.0
        if self.state == 'detected':
            self.detect_timer += dt
            if self.detect_timer >= cfg.CLASSIFY_TIME:
                self.state = 'classified'

    def draw(self, surface, fonts):
        """Малювання цілі залежно від стану."""
        if self.state == 'undetected':
            return

        x, y = int(self.x), int(self.y)

        if self.state == 'detected':
            self._draw_detected(surface, x, y)
        elif self.state == 'classified':
            self._draw_classified(surface, x, y, fonts)
        elif self.state == 'destroyed':
            self._draw_destroyed(surface, x, y, fonts)

        # Рамка виділення
        if self.selected:
            pygame.draw.circle(surface, (255, 255, 255), (x, y), 20, 2)

    # ========== Стани відображення ==========

    def _draw_detected(self, surface, x, y):
        """Пульсуюча червона точка з колом — виявлена, не класифікована."""
        pulse = abs(math.sin(self.pulse_phase))
        radius = int(5 + 4 * pulse)
        pygame.draw.circle(surface, (255, 60, 60), (x, y), radius)
        outer = radius + 6
        # Зовнішнє коло
        ring_surf = pygame.Surface((outer * 2, outer * 2), pygame.SRCALPHA)
        pygame.draw.circle(ring_surf, (255, 100, 100, 120),
                           (outer, outer), outer, 1)
        surface.blit(ring_surf, (x - outer, y - outer))

    def _draw_classified(self, surface, x, y, fonts):
        """Іконка типу цілі з підписом — класифікована.

        Підпис показує фіксовану к-сть дронів для знищення.
        """
        self._draw_icon(surface, x, y, 1.0)
        text = f"{self.label} ({self.drones_needed})"
        txt_surf = fonts['small'].render(text, True, (255, 200, 200))
        surface.blit(txt_surf, (x - txt_surf.get_width() // 2, y + 16))

    def _draw_destroyed(self, surface, x, y, fonts):
        """Напівпрозора іконка з галочкою — знищена."""
        self._draw_icon(surface, x, y, 0.35)
        # Галочка
        pygame.draw.line(surface, (80, 220, 80), (x - 7, y + 1), (x - 2, y + 7), 3)
        pygame.draw.line(surface, (80, 220, 80), (x - 2, y + 7), (x + 8, y - 5), 3)

    # ========== Іконки типів ==========

    def _draw_icon(self, surface, x, y, brightness):
        """Розподіл малювання іконки за типом."""
        if self.type == 'infantry':
            self._draw_infantry(surface, x, y, brightness)
        elif self.type == 'vehicle':
            self._draw_vehicle(surface, x, y, brightness)
        elif self.type == 'bmp':
            self._draw_bmp(surface, x, y, brightness)
        elif self.type == 'tank':
            self._draw_tank(surface, x, y, brightness)

    def _draw_infantry(self, surface, x, y, b):
        """Солдатик — піхота."""
        c = (int(220 * b), int(70 * b), int(70 * b))
        pygame.draw.circle(surface, c, (x, y - 9), 4)        # голова
        pygame.draw.line(surface, c, (x, y - 5), (x, y + 3), 2)    # тіло
        pygame.draw.line(surface, c, (x - 6, y - 2), (x + 6, y - 2), 2)  # руки
        pygame.draw.line(surface, c, (x, y + 3), (x - 5, y + 11), 2)     # нога Л
        pygame.draw.line(surface, c, (x, y + 3), (x + 5, y + 11), 2)     # нога П

    def _draw_vehicle(self, surface, x, y, b):
        """Прямокутник з колесами — авто."""
        c = (int(220 * b), int(130 * b), int(50 * b))
        pygame.draw.rect(surface, c, (x - 11, y - 6, 22, 12), 2)  # корпус
        pygame.draw.circle(surface, c, (x - 7, y + 7), 3)          # колесо Л
        pygame.draw.circle(surface, c, (x + 7, y + 7), 3)          # колесо П

    def _draw_bmp(self, surface, x, y, b):
        """Прямокутник з колесами і баштою — БМП."""
        c = (int(200 * b), int(140 * b), int(50 * b))
        pygame.draw.rect(surface, c, (x - 13, y - 6, 26, 14), 2)   # корпус
        for dx in [-8, 0, 8]:
            pygame.draw.circle(surface, c, (x + dx, y + 8), 3)      # колеса
        pygame.draw.rect(surface, c, (x - 5, y - 11, 10, 6))       # башта

    def _draw_tank(self, surface, x, y, b):
        """Корпус з гарматою — танк."""
        c = (int(200 * b), int(90 * b), int(40 * b))
        pygame.draw.rect(surface, c, (x - 14, y - 5, 28, 14), 2)   # гусениці
        pygame.draw.circle(surface, c, (x - 2, y + 1), 7)           # башта
        pygame.draw.line(surface, c, (x + 2, y - 1), (x + 18, y - 5), 3)  # гармата

    # ========== Утиліти ==========

    def get_zone(self):
        """Координати зони (col, row) де розташована ціль."""
        col = max(0, min(cfg.GRID_COLS - 1, int(self.x // cfg.ZONE_W)))
        row = max(0, min(cfg.GRID_ROWS - 1, int(self.y // cfg.ZONE_H)))
        return (col, row)
