# metrics.py — Збір та візуалізація метрик симуляції рою БПЛА

import os
import csv
import datetime
import config as cfg


class MetricsCollector:
    """Збір метрик симуляції в реальному часі та генерація звітів."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Скидання всіх зібраних метрик."""
        self.mission_start_time = 0.0
        self.mission_end_time = None

        # Покриття карти по часу — (час, відсоток)
        self.coverage_timeline = []

        # Активні дрони по часу — (час, кількість)
        self.active_drones_timeline = []

        # Події цілей — target_id -> словник
        self.target_events = {}

        # Втрати дронів — (час, id, причина)
        self.drone_loss_events = []
        self.losses_by_reason = {'reb': 0, 'kamikaze': 0, 'battery': 0}

        # Консенсус
        self.consensus_rounds = 0
        self.consensus_times = []

        # Аукціони — (час, label, target_id, {drone_id: bid}, [winner_ids])
        self.auction_log = []

        # Перекриття зон — множини drone_id для кожної зони
        self.zone_visitors = [
            [set() for _ in range(cfg.GRID_COLS)]
            for _ in range(cfg.GRID_ROWS)
        ]

        # Час відновлення рою після втрати
        self.swarm_recovery_times = []       # (loss_time, recovery_time, delta)
        self._pending_recoveries = {}        # target_id -> loss_time

        # Внутрішній таймер збору
        self._sample_timer = 0.0

    # ==================== ПЕРІОДИЧНИЙ ЗБІР ====================

    def update(self, sim_time, dt, drones, coverage):
        """Збір покриття та активних дронів кожні 2 секунди."""
        self._sample_timer += dt
        if self._sample_timer < 2.0:
            return
        self._sample_timer = 0.0

        # Покриття
        total = cfg.GRID_COLS * cfg.GRID_ROWS
        covered = sum(
            1 for r in range(cfg.GRID_ROWS)
            for c in range(cfg.GRID_COLS)
            if coverage[r][c]
        )
        pct = round(covered / total * 100, 1)
        self.coverage_timeline.append((round(sim_time, 1), pct))

        # Активні дрони
        active = sum(1 for d in drones if d.status != 'lost')
        self.active_drones_timeline.append((round(sim_time, 1), active))

    # ==================== ЗАПИС ПОДІЙ ====================

    def record_zone_visit(self, row, col, drone_id):
        """Зафіксувати відвідування зони конкретним дроном."""
        if 0 <= row < cfg.GRID_ROWS and 0 <= col < cfg.GRID_COLS:
            self.zone_visitors[row][col].add(drone_id)

    def record_target_detected(self, sim_time, target):
        """Зафіксувати момент виявлення цілі."""
        self.target_events[target.id] = {
            'type': target.label,
            'detect_time': round(sim_time, 2),
            'classify_time': None,
            'destroy_time': None,
            'drones': [],
        }

    def record_target_classified(self, sim_time, target):
        """Зафіксувати момент класифікації цілі."""
        if target.id in self.target_events:
            self.target_events[target.id]['classify_time'] = round(sim_time, 2)

    def record_target_destroyed(self, sim_time, target):
        """Зафіксувати момент знищення цілі."""
        if target.id in self.target_events:
            ev = self.target_events[target.id]
            ev['destroy_time'] = round(sim_time, 2)
            ev['drones'] = list(target.assigned_drones)

        # Якщо була очікувана recovery для цієї цілі — зафіксувати
        if target.id in self._pending_recoveries:
            loss_t = self._pending_recoveries.pop(target.id)
            delta = round(sim_time - loss_t, 2)
            self.swarm_recovery_times.append((loss_t, round(sim_time, 2), delta))

    def record_drone_loss(self, sim_time, drone, lost_reason):
        """Зафіксувати втрату дрона з причиною.

        lost_reason: 'ew' | 'kamikaze' | 'battery' (drone.lost_reason).
        """
        reason_map = {'ew': 'reb', 'kamikaze': 'kamikaze', 'battery': 'battery'}
        key = reason_map.get(lost_reason, 'battery')
        self.losses_by_reason[key] += 1
        self.drone_loss_events.append((round(sim_time, 2), drone.id, key))

        # Якщо дрон атакував ціль — відкрити recovery-таймер
        if drone.target_obj is not None and drone.target_obj.state == 'classified':
            self._pending_recoveries[drone.target_obj.id] = sim_time

    def record_consensus_round(self, sim_time):
        """Зафіксувати раунд консенсусу."""
        self.consensus_rounds += 1
        self.consensus_times.append(round(sim_time, 2))

    def record_auction(self, sim_time, target, all_bids, winner_ids):
        """Зафіксувати результат аукціону.

        all_bids: [(drone_id, bid_value), ...]
        winner_ids: [drone_id, ...]
        """
        bids_dict = {did: bid for did, bid in all_bids}
        self.auction_log.append((
            round(sim_time, 2),
            target.label,
            target.id,
            bids_dict,
            list(winner_ids),
        ))

        # Якщо це переназначення після втрати — recovery
        if target.id in self._pending_recoveries:
            loss_t = self._pending_recoveries.pop(target.id)
            delta = round(sim_time - loss_t, 2)
            self.swarm_recovery_times.append((loss_t, round(sim_time, 2), delta))

    # ==================== ВЛАСТИВОСТІ ====================

    @property
    def coverage_overlap(self):
        """Кількість зон, покритих більше ніж одним дроном."""
        return sum(
            1 for r in range(cfg.GRID_ROWS)
            for c in range(cfg.GRID_COLS)
            if len(self.zone_visitors[r][c]) > 1
        )

    # ==================== EXPORT CSV ====================

    def export_to_csv(self, directory='results'):
        """Збереження всіх метрик у CSV файли.

        Кожен прогін — окрема підпапка results/{scenario}_{timestamp},
        тож повторні запуски тієї ж стратегії не перезаписують одне одного.
        Повертає шлях до створеної підпапки.
        """
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        run_dir = os.path.join(directory, f'{cfg.SCENARIO}_{ts}')
        os.makedirs(run_dir, exist_ok=True)
        sc = cfg.SCENARIO

        # 1. Покриття та активні дрони
        path = os.path.join(run_dir, f'{sc}_coverage.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['Час (с)', 'Покриття (%)', 'Активних дронів'])
            for i, (t, pct) in enumerate(self.coverage_timeline):
                active = (self.active_drones_timeline[i][1]
                          if i < len(self.active_drones_timeline) else '')
                w.writerow([t, pct, active])

        # 2. Цілі
        path = os.path.join(run_dir, f'{sc}_targets.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['ID', 'Тип', 'Виявлення (с)', 'Класифікація (с)',
                         'Знищення (с)', 'Дрони'])
            for tid in sorted(self.target_events):
                ev = self.target_events[tid]
                drones_str = ';'.join(str(d) for d in ev['drones'])
                w.writerow([tid, ev['type'], ev['detect_time'],
                            ev['classify_time'], ev['destroy_time'],
                            drones_str])

        # 3. Втрати дронів
        path = os.path.join(run_dir, f'{sc}_drone_losses.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['Час (с)', 'ID дрона', 'Причина'])
            for t, did, reason in self.drone_loss_events:
                w.writerow([t, did, reason])

        # 4. Аукціони
        path = os.path.join(run_dir, f'{sc}_auctions.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['Час (с)', 'Ціль', 'ID цілі',
                         'Bid значення', 'Переможці'])
            for t, label, tid, bids, winners in self.auction_log:
                bids_str = '; '.join(
                    f'D{d}={b:.2f}' for d, b in sorted(bids.items()))
                winners_str = ', '.join(f'D{wid}' for wid in winners)
                w.writerow([t, label, tid, bids_str, winners_str])

        # 5. Зведення
        path = os.path.join(run_dir, f'{sc}_summary.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['Параметр', 'Значення'])
            w.writerow(['Сценарій', sc])
            w.writerow(['MAP_SEED', cfg.MAP_SEED])
            w.writerow(['Раундів консенсусу', self.consensus_rounds])
            w.writerow(['Втрат дронів', len(self.drone_loss_events)])
            w.writerow(['Втрат від РЕБ', self.losses_by_reason['reb']])
            w.writerow(['Втрат камікадзе (продуктивних)',
                        self.losses_by_reason['kamikaze']])
            w.writerow(['Втрат від батареї (змарнованих)',
                        self.losses_by_reason['battery']])
            destroyed = sum(1 for ev in self.target_events.values()
                            if ev['destroy_time'] is not None)
            w.writerow(['Знищено цілей', destroyed])
            kamikaze = self.losses_by_reason['kamikaze']
            drones_per_target = (round(kamikaze / destroyed, 2)
                                 if destroyed > 0 else 'N/A')
            w.writerow(['Дронів на знищену ціль', drones_per_target])
            w.writerow(['Аукціонів проведено', len(self.auction_log)])
            w.writerow(['Перекриття зон (>1 дрон)', self.coverage_overlap])
            if self.coverage_timeline:
                w.writerow(['Фінальне покриття (%)',
                            self.coverage_timeline[-1][1]])
            if self.swarm_recovery_times:
                avg = sum(d for _, _, d in self.swarm_recovery_times) \
                      / len(self.swarm_recovery_times)
                w.writerow(['Середній час відновлення (с)', round(avg, 2)])
            else:
                w.writerow(['Середній час відновлення (с)', 'N/A'])
            if self.mission_end_time is not None:
                w.writerow(['Час місії (с)',
                            round(self.mission_end_time - self.mission_start_time, 1)])

        return run_dir

    # ==================== MATPLOTLIB ЗВІТ ====================

    def generate_report(self, directory='results'):
        """Генерація 4 графіків та збереження як PNG."""
        try:
            import matplotlib
            import matplotlib.pyplot as plt
        except ImportError:
            print("[metrics] matplotlib не встановлено. "
                  "Встановіть: pip install matplotlib")
            return

        # Підтримка кирилиці
        plt.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.sans-serif'] = [
            'Arial', 'Segoe UI', 'DejaVu Sans']

        os.makedirs(directory, exist_ok=True)
        sc = cfg.SCENARIO

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(
            f'Метрики симуляції рою БПЛА — сценарій: {sc}',
            fontsize=15, fontweight='bold')

        self._plot_coverage(axes[0, 0])
        self._plot_targets(axes[0, 1])
        self._plot_active_drones(axes[1, 0])
        self._plot_auctions(axes[1, 1])

        plt.tight_layout(rect=[0, 0, 1, 0.94])

        path = os.path.join(directory, f'{sc}_report.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        print(f'[metrics] Звіт збережено: {os.path.abspath(path)}')
        plt.close(fig)

    # ---------- Графік 1: Покриття карти ----------

    def _plot_coverage(self, ax):
        """Лінійний графік покриття карти (%) по часу."""
        if not self.coverage_timeline:
            ax.text(0.5, 0.5, 'Немає даних', ha='center', va='center',
                    fontsize=12, color='gray')
            ax.set_title('Покриття карти')
            return

        times = [t for t, _ in self.coverage_timeline]
        pcts = [p for _, p in self.coverage_timeline]

        ax.plot(times, pcts, color='#2ecc71', linewidth=2.2,
                marker='o', markersize=3, label='Покриття')
        ax.fill_between(times, pcts, alpha=0.12, color='#2ecc71')

        # Вертикальні лінії — моменти втрат дронів
        for t, did, reason in self.drone_loss_events:
            ax.axvline(x=t, color='#e74c3c', linestyle='--',
                       alpha=0.5, linewidth=0.9)

        ax.set_xlabel('Час (с)')
        ax.set_ylabel('Покриття (%)')
        ax.set_title('Покриття карти')
        ax.set_ylim(0, 105)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9, loc='lower right')

    # ---------- Графік 2: Часовий профіль цілей ----------

    def _plot_targets(self, ax):
        """Горизонтальний стовпчастий: виявлення → класифікація → знищення."""
        if not self.target_events:
            ax.text(0.5, 0.5, 'Немає даних', ha='center', va='center',
                    fontsize=12, color='gray')
            ax.set_title('Часовий профіль цілей')
            return

        labels = []
        phase_detect = []
        phase_classify = []
        phase_destroy = []

        for tid in sorted(self.target_events):
            ev = self.target_events[tid]
            labels.append(f"#{tid} {ev['type']}")

            dt = ev['detect_time'] if ev['detect_time'] is not None else 0
            ct = ev['classify_time'] if ev['classify_time'] is not None else dt
            dest = ev['destroy_time'] if ev['destroy_time'] is not None else ct

            phase_detect.append(dt)
            phase_classify.append(ct - dt)
            phase_destroy.append(dest - ct)

        y_pos = list(range(len(labels)))

        ax.barh(y_pos, phase_detect,
                color='#3498db', label='До виявлення')
        ax.barh(y_pos, phase_classify, left=phase_detect,
                color='#f39c12', label='Класифікація')
        lefts = [d + c for d, c in zip(phase_detect, phase_classify)]
        ax.barh(y_pos, phase_destroy, left=lefts,
                color='#e74c3c', label='Знищення')

        ax.set_yticks(y_pos)
        ax.set_yticklabels(labels, fontsize=9)
        ax.set_xlabel('Час (с)')
        ax.set_title('Часовий профіль цілей')
        ax.legend(fontsize=8, loc='lower right')
        ax.grid(True, alpha=0.3, axis='x')

    # ---------- Графік 3: Активні дрони ----------

    def _plot_active_drones(self, ax):
        """Лінійний графік кількості активних дронів по часу."""
        if not self.active_drones_timeline:
            ax.text(0.5, 0.5, 'Немає даних', ha='center', va='center',
                    fontsize=12, color='gray')
            ax.set_title('Динаміка активних дронів')
            return

        times = [t for t, _ in self.active_drones_timeline]
        counts = [c for _, c in self.active_drones_timeline]

        ax.plot(times, counts, color='#3498db', linewidth=2.2,
                marker='s', markersize=3, label='Активні дрони')
        ax.fill_between(times, counts, alpha=0.12, color='#3498db')

        # Маркери втрат
        for lt, did, reason in self.drone_loss_events:
            # Знайти найближчий відлік
            closest = cfg.NUM_DRONES
            for tm, c in self.active_drones_timeline:
                if tm >= lt:
                    closest = c
                    break
            marker = 'X' if reason == 'reb' else ('*' if reason == 'kamikaze' else 'v')
            label_txt = f'D{did} ({reason})'
            ax.plot(lt, closest, marker, color='#e74c3c',
                    markersize=10, markeredgewidth=2, label=label_txt)

        ax.set_xlabel('Час (с)')
        ax.set_ylabel('Кількість дронів')
        ax.set_title('Динаміка активних дронів')
        ax.set_ylim(0, cfg.NUM_DRONES + 1)
        ax.grid(True, alpha=0.3)
        # Уникнення дублювання легенди
        handles, lbls = ax.get_legend_handles_labels()
        by_label = dict(zip(lbls, handles))
        ax.legend(by_label.values(), by_label.keys(),
                  fontsize=8, loc='lower left')

    # ---------- Графік 4: Bid в аукціонах ----------

    def _plot_auctions(self, ax):
        """Стовпчастий графік порівняння bid значень в аукціонах."""
        if not self.auction_log:
            ax.text(0.5, 0.5, 'Аукціони не проводились',
                    ha='center', va='center', fontsize=12, color='gray')
            ax.set_title('Bid значення в аукціонах')
            return

        n = len(self.auction_log)
        x_labels = []

        for idx, (t, label, tid, bids, winners) in enumerate(self.auction_log):
            x_labels.append(f"{label}\n{t}с")

            sorted_bids = sorted(bids.items(), key=lambda x: x[1],
                                 reverse=True)
            count = max(len(sorted_bids), 1)
            bar_w = 0.7 / count

            for j, (did, bid) in enumerate(sorted_bids):
                x_pos = idx + (j - count / 2 + 0.5) * bar_w
                is_winner = did in winners
                color = '#2ecc71' if is_winner else '#bdc3c7'
                edge = '#27ae60' if is_winner else '#95a5a6'
                ax.bar(x_pos, bid, width=bar_w * 0.88,
                       color=color, edgecolor=edge, linewidth=0.6)
                ax.text(x_pos, bid + 0.008, f'D{did}',
                        ha='center', fontsize=6, rotation=45)

        ax.set_xticks(range(n))
        ax.set_xticklabels(x_labels, fontsize=7)
        ax.set_ylabel('Bid')
        ax.set_title('Bid значення в аукціонах')
        ax.grid(True, alpha=0.3, axis='y')
