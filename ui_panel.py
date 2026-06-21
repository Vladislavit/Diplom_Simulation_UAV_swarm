# ui_panel.py — Права панель: заголовок, статистика, список дронів, лог комунікації

import pygame
import config as cfg


class UIPanel:
    """Права інформаційна панель 340px."""

    def __init__(self):
        self.font_title = pygame.font.SysFont('arial', 18, bold=True)
        self.font_section = pygame.font.SysFont('arial', 14, bold=True)
        self.font_normal = pygame.font.SysFont('arial', 13)
        self.font_small = pygame.font.SysFont('arial', 11)
        self.font_log = pygame.font.SysFont('arial', 11)

        self.panel_x = cfg.MAP_WIDTH
        self.panel_w = cfg.PANEL_WIDTH
        self.log_messages = []   # [(time_str, message, msg_type)]
        self.max_log = 15

    # ========== ЛОГ ==========

    def add_log(self, time_str, message, msg_type='info'):
        """Додати повідомлення до логу комунікації."""
        self.log_messages.append((time_str, message, msg_type))
        # Обмеження розміру буфера
        if len(self.log_messages) > 60:
            self.log_messages = self.log_messages[-60:]

    def _log_color(self, msg_type):
        """Колір повідомлення за типом."""
        return {
            'consensus': cfg.COLOR_LOG_CONSENSUS,
            'auction':   cfg.COLOR_LOG_AUCTION,
            'threat':    cfg.COLOR_LOG_THREAT,
            'warning':   cfg.COLOR_LOG_WARNING,
            'info':      cfg.COLOR_LOG_INFO,
        }.get(msg_type, cfg.COLOR_LOG_INFO)

    # ========== МАЛЮВАННЯ ПАНЕЛІ ==========

    def draw(self, screen, sim_time, drones, targets, coverage,
             selected_drone, selected_target):
        """Повне малювання правої панелі."""
        # Фон
        panel = pygame.Rect(self.panel_x, 0, self.panel_w, cfg.WINDOW_HEIGHT)
        pygame.draw.rect(screen, cfg.COLOR_PANEL_BG, panel)
        pygame.draw.line(screen, cfg.COLOR_PANEL_BORDER,
                         (self.panel_x, 0),
                         (self.panel_x, cfg.WINDOW_HEIGHT), 2)

        x = self.panel_x + 12
        w = self.panel_w - 24
        y = 10

        # --- Заголовок ---
        y = self._draw_header(screen, x, y, w, sim_time)

        # --- Статистика ---
        y = self._draw_stats(screen, x, y, w, drones, targets, coverage)

        # --- Список дронів ---
        y = self._draw_drone_list(screen, x, y, w, drones)

        # --- Деталі виділеного об'єкта ---
        if selected_drone is not None:
            y = self._draw_separator(screen, x, y, w)
            y = self._draw_drone_details(screen, x, y, selected_drone)
        elif selected_target is not None:
            y = self._draw_separator(screen, x, y, w)
            y = self._draw_target_details(screen, x, y, selected_target)

        # --- Лог комунікації ---
        y = self._draw_separator(screen, x, y, w)
        self._draw_log(screen, x, y, w)

    # ========== СЕКЦІЇ ==========

    def _draw_header(self, screen, x, y, w, sim_time):
        """Заголовок та час місії."""
        title = self.font_title.render('Симуляція рою БПЛА', True,
                                       cfg.COLOR_TEXT_BRIGHT)
        screen.blit(title, (x, y))
        y += 24

        minutes = int(sim_time) // 60
        seconds = int(sim_time) % 60
        time_txt = self.font_normal.render(
            f"Час місії: {minutes:02d}:{seconds:02d}", True, cfg.COLOR_TEXT_DIM)
        screen.blit(time_txt, (x, y))
        y += 20

        return self._draw_separator(screen, x, y, w)

    def _draw_stats(self, screen, x, y, w, drones, targets, coverage):
        """Блок статистики."""
        label = self.font_section.render('Статистика', True,
                                         cfg.COLOR_TEXT_BRIGHT)
        screen.blit(label, (x, y))
        y += 20

        active = sum(1 for d in drones if d.status != 'lost')
        total_zones = cfg.GRID_COLS * cfg.GRID_ROWS
        covered = sum(
            1 for r in range(cfg.GRID_ROWS)
            for c in range(cfg.GRID_COLS)
            if coverage[r][c]
        )
        pct = int(covered / total_zones * 100) if total_zones > 0 else 0
        destroyed = sum(1 for t in targets if t.state == 'destroyed')
        ew_count = sum(1 for d in drones
                       if d.status == 'lost' and d.loss_reported)

        stats = [
            f"Активних дронів: {active}/{len(drones)}",
            f"Покриття карти: {pct}%",
            f"Цілей знищено: {destroyed}/{len(targets)}",
            f"РЕБ загрози: {ew_count}",
        ]
        for s in stats:
            surf = self.font_small.render(s, True, cfg.COLOR_TEXT)
            screen.blit(surf, (x + 4, y))
            y += 16

        y += 4
        return self._draw_separator(screen, x, y, w)

    def _draw_drone_list(self, screen, x, y, w, drones):
        """Список дронів: крапка статусу, ID, текст, батарея."""
        label = self.font_section.render('Дрони', True,
                                         cfg.COLOR_TEXT_BRIGHT)
        screen.blit(label, (x, y))
        y += 19

        for drone in drones:
            color = drone.get_color()

            # Кольорова крапка статусу
            pygame.draw.circle(screen, color, (x + 6, y + 7), 4)

            # ID та статус (мітка [L] для лідера у leader_follower)
            lead = " [L]" if getattr(drone, 'is_leader', False) else ""
            txt = f"D{drone.id}{lead} {drone.get_status_text()}"
            tc = cfg.COLOR_TEXT if drone.status != 'lost' else cfg.COLOR_TEXT_DIM
            surf = self.font_small.render(txt, True, tc)
            screen.blit(surf, (x + 14, y))

            # Індикатор заряду батареї
            bar_x = x + w - 54
            bar_y = y + 3
            bar_w = 50
            bar_h = 9
            # Фон
            pygame.draw.rect(screen, (45, 45, 55),
                             (bar_x, bar_y, bar_w, bar_h))
            # Заповнення
            fill = int(bar_w * max(0, drone.battery) / 100)
            if drone.battery > 50:
                bc = (50, 180, 50)
            elif drone.battery > cfg.LOW_BATTERY:
                bc = (210, 175, 40)
            else:
                bc = (210, 55, 55)
            if fill > 0:
                pygame.draw.rect(screen, bc,
                                 (bar_x, bar_y, fill, bar_h))
            # Рамка
            pygame.draw.rect(screen, (75, 75, 85),
                             (bar_x, bar_y, bar_w, bar_h), 1)

            y += 17

        y += 4
        return y

    def _draw_drone_details(self, screen, x, y, drone):
        """Деталі виділеного дрона."""
        label = self.font_section.render(
            f'Дрон D{drone.id}', True, cfg.COLOR_TEXT_BRIGHT)
        screen.blit(label, (x, y))
        y += 18

        details = [
            f"Статус: {drone.get_status_text()}",
            f"Позиція: ({int(drone.x)}, {int(drone.y)})",
            f"Батарея: {drone.battery:.1f}%",
            f"Зона: {drone.get_zone()}",
            f"Сусідів: {len(drone.neighbors)}",
            f"Bid: {drone.bid:.2f}",
        ]
        for d in details:
            surf = self.font_small.render(d, True, cfg.COLOR_TEXT)
            screen.blit(surf, (x + 4, y))
            y += 15
        y += 4
        return y

    def _draw_target_details(self, screen, x, y, target):
        """Деталі виділеної цілі."""
        label = self.font_section.render(
            f'Ціль #{target.id}', True, cfg.COLOR_TEXT_BRIGHT)
        screen.blit(label, (x, y))
        y += 18

        state_names = {
            'undetected': 'Невиявлена',
            'detected': 'Виявлена',
            'classified': 'Класифікована',
            'destroyed': 'Знищена',
        }
        details = [
            f"Тип: {target.label}",
            f"Стан: {state_names.get(target.state, target.state)}",
            f"Потрібно дронів: {target.drones_needed}",
            f"Призначено: {target.assigned_drones}",
        ]
        for d in details:
            surf = self.font_small.render(d, True, cfg.COLOR_TEXT)
            screen.blit(surf, (x + 4, y))
            y += 15
        y += 4
        return y

    def _draw_log(self, screen, x, y, w):
        """Лог комунікації рою — останні повідомлення."""
        label = self.font_section.render('Лог комунікації', True,
                                         cfg.COLOR_TEXT_BRIGHT)
        screen.blit(label, (x, y))
        y += 18

        visible = self.log_messages[-self.max_log:]
        for time_s, msg, mtype in visible:
            color = self._log_color(mtype)
            line = f"[{time_s}] {msg}"
            # Обрізка довгих рядків
            if len(line) > 44:
                line = line[:42] + ".."
            surf = self.font_log.render(line, True, color)
            screen.blit(surf, (x + 2, y))
            y += 14
            if y > cfg.WINDOW_HEIGHT - 8:
                break

    # ========== УТИЛІТИ ==========

    def _draw_separator(self, screen, x, y, w):
        """Горизонтальний роздільник."""
        pygame.draw.line(screen, cfg.COLOR_PANEL_BORDER,
                         (x, y), (x + w, y), 1)
        return y + 7
