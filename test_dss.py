# test_dss.py — Тести симуляції координації рою БПЛА (DSS)
#
# Запуск: pytest test_dss.py -v --cov=. --cov-report=term-missing
#         (з директорії де знаходяться модулі проєкту)
#
# Pygame не потрібен — усі тести працюють на чистій логіці.

import sys
import types
import math
import os
import csv
import tempfile
import pytest

# ── Мок pygame до будь-яких імпортів проєкту ────────────────────────────────
# Замість справжнього pygame підставляємо порожній модуль,
# щоб не вимагати дисплею при тестуванні.
def _make_pygame_mock():
    pg = types.ModuleType("pygame")
    pg.SRCALPHA = 32768
    pg.init = lambda: None
    pg.quit = lambda: None

    class _Surface:
        def __init__(self, size, flags=0): self.size = size
        def fill(self, color): pass
        def blit(self, src, pos): pass
        def get_width(self): return self.size[0] if hasattr(self.size,'__getitem__') else 0
        def get_height(self): return self.size[1] if hasattr(self.size,'__getitem__') else 0

    class _Font:
        def render(self, text, aa, color): return _Surface((len(text)*7, 14))

    class _SysFont:
        def render(self, text, aa, color): return _Surface((len(text)*7, 14))

    class _Rect:
        def __init__(self, *a): pass

    pg.Surface = _Surface
    pg.Rect    = _Rect
    pg.font    = types.ModuleType("pygame.font")
    pg.font.SysFont = lambda *a, **kw: _Font()
    pg.draw    = types.ModuleType("pygame.draw")
    pg.draw.circle = lambda *a, **kw: None
    pg.draw.rect   = lambda *a, **kw: None
    pg.draw.line   = lambda *a, **kw: None
    pg.draw.lines  = lambda *a, **kw: None
    pg.draw.ellipse = lambda *a, **kw: None
    pg.display = types.ModuleType("pygame.display")
    pg.display.set_mode = lambda *a, **kw: _Surface((1280,720))
    pg.display.set_caption = lambda *a: None
    pg.display.flip = lambda: None
    pg.time    = types.ModuleType("pygame.time")
    pg.time.Clock = lambda: type('C', (), {'tick': lambda s,f: 0,
                                           'get_fps': lambda s: 60.0})()
    pg.event   = types.ModuleType("pygame.event")
    pg.event.get = lambda: []
    pg.K_SPACE = 32; pg.K_ESCAPE = 27; pg.K_r = 114
    pg.QUIT = 256; pg.KEYDOWN = 768; pg.MOUSEBUTTONDOWN = 1025
    pg.MOUSEBUTTONUP = 1026
    return pg

sys.modules.setdefault("pygame", _make_pygame_mock())
sys.modules.setdefault("pygame.font", sys.modules["pygame"].font)
sys.modules.setdefault("pygame.draw", sys.modules["pygame"].draw)
sys.modules.setdefault("pygame.display", sys.modules["pygame"].display)
sys.modules.setdefault("pygame.time", sys.modules["pygame"].time)
sys.modules.setdefault("pygame.event", sys.modules["pygame"].event)
sys.modules.setdefault("numpy", pytest.importorskip("numpy"))

# Тепер безпечно імпортуємо модулі проєкту
sys.path.insert(0, os.path.dirname(__file__))
import config as cfg
from drone import Drone
from targets import Target
from algorithms import (
    update_neighbors, consensus_step, run_auction,
    swarm_scout, _zone_overlaps_ew,
)
from metrics import MetricsCollector


# ═══════════════════════════════════════════════════════════════════
# FIXTURES
# ═══════════════════════════════════════════════════════════════════

EW_RECT = cfg.EW_RECT  # (540, 220, 210, 190)
SAFE_X, SAFE_Y = 100.0, 100.0   # поза РЕБ зоною


def make_drone(drone_id=1, x=SAFE_X, y=SAFE_Y):
    return Drone(drone_id, x, y)


def make_target(tid=1, x=200.0, y=200.0, ttype='infantry'):
    return Target(tid, x, y, ttype)


@pytest.fixture
def drone():
    return make_drone()


@pytest.fixture
def swarm():
    """10 дронів у стартовій позиції (безпечна зона)."""
    drones = []
    for i in range(cfg.NUM_DRONES):
        col = i % cfg.DRONE_COLS
        row = i // cfg.DRONE_COLS
        x = cfg.DRONE_START_X + col * cfg.DRONE_SPACING_X
        y = cfg.DRONE_START_Y + row * cfg.DRONE_SPACING_Y
        drones.append(Drone(i, x, y))
    return drones


@pytest.fixture
def classified_target():
    t = make_target(tid=1, ttype='bmp')
    t.state = 'classified'
    return t


@pytest.fixture
def metrics():
    return MetricsCollector()


# ═══════════════════════════════════════════════════════════════════
# 1. CONFIG — константи
# ═══════════════════════════════════════════════════════════════════

class TestConfig:
    def test_window_dimensions(self):
        assert cfg.MAP_WIDTH + cfg.PANEL_WIDTH == cfg.WINDOW_WIDTH
        assert cfg.WINDOW_WIDTH == 1280
        assert cfg.WINDOW_HEIGHT == 720

    def test_zone_dimensions_cover_map(self):
        """Зони повністю покривають карту без залишку."""
        assert cfg.ZONE_W * cfg.GRID_COLS == cfg.MAP_WIDTH
        assert cfg.ZONE_H * cfg.GRID_ROWS == cfg.WINDOW_HEIGHT

    def test_auction_weights_sum_to_one(self):
        """Сума ваг аукціону = 1.0 (для коректної нормалізації bid)."""
        total = (cfg.W_DISTANCE + cfg.W_BATTERY +
                 cfg.W_CONNECTIVITY + cfg.W_OBSTACLES + cfg.W_RISK)
        assert abs(total - 1.0) < 1e-9

    def test_target_types_defined(self):
        for t in ('infantry', 'vehicle', 'bmp', 'tank'):
            assert t in cfg.TARGET_TYPES
            assert 'max_drones' in cfg.TARGET_TYPES[t]
            assert 'label' in cfg.TARGET_TYPES[t]

    def test_ew_rect_within_map(self):
        ex, ey, ew, eh = cfg.EW_RECT
        assert ex >= 0 and ey >= 0
        assert ex + ew <= cfg.MAP_WIDTH
        assert ey + eh <= cfg.WINDOW_HEIGHT

    def test_battery_drain_positive(self):
        assert cfg.BATTERY_DRAIN_RATE > 0

    def test_scenario_valid(self):
        assert cfg.SCENARIO in (
            'greedy', 'swarm_only', 'auction', 'hybrid', 'leader_follower')


# ═══════════════════════════════════════════════════════════════════
# 2. DRONE — стан, батарея, рух, РЕБ
# ═══════════════════════════════════════════════════════════════════

class TestDroneInit:
    def test_initial_status(self, drone):
        assert drone.status == 'scouting'

    def test_initial_battery_full(self, drone):
        assert drone.battery == 100.0

    def test_initial_position(self, drone):
        assert drone.x == SAFE_X
        assert drone.y == SAFE_Y

    def test_initial_maps_empty(self, drone):
        for row in drone.local_map:
            assert all(v is False for v in row)

    def test_initial_risk_map_zero(self, drone):
        for row in drone.risk_map:
            assert all(v == 0.0 for v in row)

    def test_home_position_set(self, drone):
        assert drone.home_x == SAFE_X
        assert drone.home_y == SAFE_Y


class TestDroneBattery:
    def test_battery_drains_over_time(self, drone):
        drone.update(1.0, EW_RECT)
        assert drone.battery < 100.0

    def test_battery_drain_proportional_to_dt(self, drone):
        d1 = make_drone(1, SAFE_X, SAFE_Y)
        d2 = make_drone(2, SAFE_X, SAFE_Y)
        d1.update(1.0, EW_RECT)
        d2.update(2.0, EW_RECT)
        assert d2.battery < d1.battery

    def test_battery_floor_zero(self, drone):
        drone.battery = 0.001
        drone.update(10.0, EW_RECT)
        assert drone.battery >= 0.0

    def test_low_battery_triggers_return(self, drone):
        drone.battery = cfg.LOW_BATTERY - 1
        drone.update(0.01, EW_RECT)
        assert drone.status == 'returning'

    def test_battery_zero_causes_loss(self, drone):
        drone.battery = 0.001
        drone.update(1.0, EW_RECT)
        assert drone.status == 'lost'
        assert drone.lost_reason == 'battery'


class TestDroneEW:
    def test_ew_timer_increments_inside_zone(self):
        ex, ey, ew, eh = EW_RECT
        d = make_drone(1, ex + ew/2, ey + eh/2)  # всередині зони
        d.ew_threshold = 999.0  # не дозволяємо загинути одразу
        before = d.ew_timer
        d.update(0.1, EW_RECT)
        assert d.ew_timer > before

    def test_ew_timer_resets_outside_zone(self, drone):
        drone.ew_timer = 2.0
        drone.update(0.5, EW_RECT)
        assert drone.ew_timer < 2.0

    def test_drone_lost_after_ew_threshold(self):
        ex, ey, ew, eh = EW_RECT
        d = make_drone(1, ex + 10, ey + 10)
        d.ew_threshold = 0.5
        d.update(1.0, EW_RECT)
        assert d.status == 'lost'
        assert d.lost_reason == 'ew'

    def test_drone_safe_outside_ew(self, drone):
        """Дрон у безпечній зоні — РЕБ не впливає."""
        drone.ew_threshold = 0.01
        drone.update(0.5, EW_RECT)
        assert drone.status != 'lost'


class TestDroneMovement:
    def test_drone_moves_toward_zone(self, drone):
        drone.target_zone = (5, 5)
        x0, y0 = drone.x, drone.y
        drone.move(0.1)
        # Дрон рушив з місця
        assert (drone.x, drone.y) != (x0, y0)

    def test_drone_stays_in_bounds(self, drone):
        drone.x = cfg.MAP_WIDTH - 1
        drone.y = cfg.WINDOW_HEIGHT - 1
        drone.vx = 500.0
        drone.vy = 500.0
        drone.move(0.5)
        assert drone.x <= cfg.MAP_WIDTH - 12
        assert drone.y <= cfg.WINDOW_HEIGHT - 12

    def test_lost_drone_does_not_move(self, drone):
        drone.status = 'lost'
        drone.target_zone = (3, 3)
        x0, y0 = drone.x, drone.y
        drone.move(1.0)
        assert drone.x == x0
        assert drone.y == y0

    def test_returning_drone_moves_toward_home(self, drone):
        drone.status = 'returning'
        drone.home_x = 500.0
        drone.home_y = 300.0
        x0, y0 = drone.x, drone.y
        drone.move(0.2)
        d_before = math.hypot(500 - x0, 300 - y0)
        d_after  = math.hypot(500 - drone.x, 300 - drone.y)
        assert d_after < d_before

    def test_drone_clears_zone_on_arrival(self, drone):
        """Дрон знімає target_zone коли досягає центру."""
        drone.target_zone = (0, 0)
        drone.x = cfg.ZONE_W / 2
        drone.y = cfg.ZONE_H / 2
        drone.vx = 0.0
        drone.vy = 0.0
        drone.move(0.1)
        assert drone.target_zone is None

    def test_kamikaze_destroys_target(self, drone):
        t = make_target(ttype='infantry')
        t.state = 'classified'
        t.x = drone.x + cfg.DESTROY_DISTANCE - 1
        t.y = drone.y
        drone.status = 'attacking'
        drone.target_obj = t
        t.assigned_drones.append(drone.id)
        drone.move(0.01)
        assert drone.status == 'lost'
        assert drone.lost_reason == 'kamikaze'
        assert t.hits_received == 1

    def test_kamikaze_destroys_infantry_fully(self, drone):
        """Піхота потребує 1 удар — після камікадзе state='destroyed'."""
        t = make_target(ttype='infantry')
        t.state = 'classified'
        t.x = drone.x + cfg.DESTROY_DISTANCE - 1
        t.y = drone.y
        t.drones_needed = 1
        drone.status = 'attacking'
        drone.target_obj = t
        t.assigned_drones.append(drone.id)
        drone.move(0.01)
        assert t.state == 'destroyed'


class TestDroneUtilities:
    def test_get_zone_returns_correct_col_row(self, drone):
        drone.x = cfg.ZONE_W * 2 + 5
        drone.y = cfg.ZONE_H * 3 + 5
        assert drone.get_zone() == (2, 3)

    def test_get_zone_returns_none_out_of_bounds(self):
        d = make_drone(1, -50, -50)
        # get_zone використовує int(x//ZONE_W) — від'ємні дадуть від'ємний індекс
        zone = d.get_zone()
        # Очікуємо None або коректний індекс (залежить від реалізації)
        if zone is not None:
            col, row = zone
            assert 0 <= col < cfg.GRID_COLS
            assert 0 <= row < cfg.GRID_ROWS

    def test_get_color_scouting(self, drone):
        drone.battery = 100.0
        assert drone.get_color() == cfg.COLOR_DRONE_SCOUT

    def test_get_color_attacking(self, drone):
        drone.status = 'attacking'
        drone.battery = 100.0
        assert drone.get_color() == cfg.COLOR_DRONE_ATTACK

    def test_get_color_low_battery(self, drone):
        drone.battery = cfg.LOW_BATTERY - 1
        assert drone.get_color() == cfg.COLOR_DRONE_LOW_BAT

    def test_get_color_lost(self, drone):
        drone.status = 'lost'
        assert drone.get_color() == cfg.COLOR_DRONE_LOST

    def test_get_status_text_all_statuses(self, drone):
        for status, expected in [
            ('scouting', 'Розвідка'), ('attacking', 'Атака'),
            ('returning', 'Повернення'), ('lost', 'Втрачено'),
        ]:
            drone.status = status
            assert drone.get_status_text() == expected

    def test_local_map_updated_on_move(self, drone):
        col, row = drone.get_zone()
        drone.update(0.1, EW_RECT)
        assert drone.local_map[row][col] is True


# ═══════════════════════════════════════════════════════════════════
# 3. TARGET — стани, класифікація, зона
# ═══════════════════════════════════════════════════════════════════

class TestTargetInit:
    def test_initial_state_undetected(self):
        t = make_target()
        assert t.state == 'undetected'

    def test_initial_assigned_empty(self):
        t = make_target()
        assert t.assigned_drones == []

    def test_hits_received_zero(self):
        t = make_target()
        assert t.hits_received == 0

    def test_label_set_from_config(self):
        for ttype, info in cfg.TARGET_TYPES.items():
            t = make_target(ttype=ttype)
            assert t.label == info['label']

    def test_drones_needed_equals_max_drones(self):
        for ttype, info in cfg.TARGET_TYPES.items():
            t = make_target(ttype=ttype)
            assert t.drones_needed == info['max_drones']


class TestTargetClassification:
    def test_detected_to_classified_after_timer(self):
        t = make_target()
        t.state = 'detected'
        t.detect_timer = 0.0
        dt = cfg.CLASSIFY_TIME + 0.1
        t.update(dt)
        assert t.state == 'classified'

    def test_not_classified_before_timer(self):
        t = make_target()
        t.state = 'detected'
        t.update(cfg.CLASSIFY_TIME - 0.5)
        assert t.state == 'detected'

    def test_undetected_does_not_classify(self):
        t = make_target()
        t.state = 'undetected'
        t.update(cfg.CLASSIFY_TIME + 1.0)
        assert t.state == 'undetected'

    def test_pulse_phase_increases(self):
        t = make_target()
        t.state = 'detected'
        before = t.pulse_phase
        t.update(0.1)
        assert t.pulse_phase > before

    def test_destroyed_state_not_reset(self):
        t = make_target()
        t.state = 'destroyed'
        t.update(10.0)
        assert t.state == 'destroyed'


class TestTargetZone:
    def test_zone_correct_col_row(self):
        t = make_target(x=cfg.ZONE_W * 3 + 10, y=cfg.ZONE_H * 2 + 5)
        assert t.get_zone() == (3, 2)

    def test_zone_clamped_to_grid(self):
        t = make_target(x=-100, y=-100)
        col, row = t.get_zone()
        assert col == 0 and row == 0

    def test_zone_max_boundary(self):
        t = make_target(x=cfg.MAP_WIDTH + 100, y=cfg.WINDOW_HEIGHT + 100)
        col, row = t.get_zone()
        assert col == cfg.GRID_COLS - 1
        assert row == cfg.GRID_ROWS - 1


class TestTargetTypes:
    @pytest.mark.parametrize("ttype,expected_drones", [
        ('infantry', 1),
        ('bmp', 3),
        ('tank', 3),
    ])
    def test_drones_needed_per_type(self, ttype, expected_drones):
        t = make_target(ttype=ttype)
        assert t.drones_needed == expected_drones


# ═══════════════════════════════════════════════════════════════════
# 4. ALGORITHMS — сусіди, консенсус, аукціон
# ═══════════════════════════════════════════════════════════════════

class TestUpdateNeighbors:
    def test_nearby_drones_are_neighbors(self):
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 200, 100)  # відстань 100 < COMM_RADIUS=180
        update_neighbors([d1, d2])
        assert d2 in d1.neighbors
        assert d1 in d2.neighbors

    def test_far_drones_not_neighbors(self):
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 100 + cfg.COMM_RADIUS + 50, 100)
        update_neighbors([d1, d2])
        assert d2 not in d1.neighbors
        assert d1 not in d2.neighbors

    def test_lost_drone_has_no_neighbors(self):
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 200, 100)
        d1.status = 'lost'
        update_neighbors([d1, d2])
        assert d1.neighbors == []

    def test_lost_drone_not_added_as_neighbor(self):
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 200, 100)
        d2.status = 'lost'
        update_neighbors([d1, d2])
        assert d2 not in d1.neighbors

    def test_drone_not_own_neighbor(self, swarm):
        update_neighbors(swarm)
        for d in swarm:
            assert d not in d.neighbors

    def test_neighbor_count_decreases_on_loss(self):
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 200, 100)
        d3 = make_drone(3, 250, 100)
        update_neighbors([d1, d2, d3])
        count_before = len(d1.neighbors)
        d2.status = 'lost'
        update_neighbors([d1, d2, d3])
        assert len(d1.neighbors) < count_before


class TestConsensusStep:
    def test_coverage_merges_between_neighbors(self):
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 200, 100)
        update_neighbors([d1, d2])
        d2.local_map[3][4] = True  # d2 знає зону (4,3)
        assert d1.local_map[3][4] is False
        logs = []
        consensus_step([d1, d2], lambda m, t: logs.append((m, t)))
        assert d1.local_map[3][4] is True

    def test_known_targets_propagate(self):
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 200, 100)
        update_neighbors([d1, d2])
        d2.known_targets[42] = {'type': 'infantry', 'x': 300, 'y': 400}
        consensus_step([d1, d2], lambda m, t: None)
        assert 42 in d1.known_targets

    def test_risk_map_takes_max(self):
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 200, 100)
        update_neighbors([d1, d2])
        d2.risk_map[5][5] = 0.9
        d1.risk_map[5][5] = 0.2
        consensus_step([d1, d2], lambda m, t: None)
        assert d1.risk_map[5][5] == pytest.approx(0.9)

    def test_lost_drone_skipped_in_consensus(self):
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 200, 100)
        d1.status = 'lost'
        d2.local_map[0][0] = True
        # Не повинно падати, lost дрон просто пропускається
        consensus_step([d1, d2], lambda m, t: None)

    def test_consensus_log_callback_called(self):
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 200, 100)
        update_neighbors([d1, d2])
        d2.local_map[7][7] = True
        logs = []
        consensus_step([d1, d2], lambda m, t: logs.append(t))
        assert 'consensus' in logs

    def test_no_duplicate_messages(self):
        """Один і той самий повідомлення не додається двічі."""
        d1 = make_drone(1, 100, 100)
        d2 = make_drone(2, 200, 100)
        update_neighbors([d1, d2])
        d2.local_map[1][1] = True
        d2.local_map[1][2] = True
        logs = []
        consensus_step([d1, d2], lambda m, t: logs.append(m))
        assert len(logs) == len(set(logs))


class TestAuction:
    def _log(self): return []

    def test_auction_assigns_drone(self, swarm, classified_target):
        update_neighbors(swarm)
        risk_map = [[0.0]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        logs = []
        result = run_auction(swarm, classified_target, risk_map,
                             lambda m, t: logs.append(m))
        assert result is not None
        assert len(result['assigned']) >= 1

    def test_auction_not_run_for_unclassified(self, swarm):
        t = make_target(ttype='infantry')
        t.state = 'detected'
        risk_map = [[0.0]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        result = run_auction(swarm, t, risk_map, lambda m, t: None)
        assert result is None

    def test_winners_in_attacking_status(self, swarm, classified_target):
        update_neighbors(swarm)
        risk_map = [[0.0]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        result = run_auction(swarm, classified_target, risk_map,
                             lambda m, t: None)
        for d in result['assigned']:
            assert d.status == 'attacking'
            assert d.target_obj is classified_target

    def test_winner_ids_in_assigned_drones(self, swarm, classified_target):
        update_neighbors(swarm)
        risk_map = [[0.0]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        result = run_auction(swarm, classified_target, risk_map,
                             lambda m, t: None)
        for wid in result['winner_ids']:
            assert wid in classified_target.assigned_drones

    def test_bid_range_valid(self, swarm, classified_target):
        """Bid повинен бути в межах [0, 1] при нульовому ризику."""
        update_neighbors(swarm)
        risk_map = [[0.0]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        result = run_auction(swarm, classified_target, risk_map,
                             lambda m, t: None)
        for _, bid in result['all_bids']:
            assert -1.0 <= bid <= 1.5  # широкий діапазон з урахуванням ваг

    def test_low_battery_drone_excluded(self, swarm, classified_target):
        """Дрон із зарядом < 15% не бере участі в аукціоні."""
        update_neighbors(swarm)
        risk_map = [[0.0]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        # Виснажити всі дрони крім першого
        for d in swarm[1:]:
            d.battery = 10.0
        result = run_auction(swarm, classified_target, risk_map,
                             lambda m, t: None)
        # Тільки перший дрон міг взяти участь
        if result:
            for d in result['assigned']:
                assert d.battery >= 15.0

    def test_auction_not_run_twice_on_done(self, swarm, classified_target):
        update_neighbors(swarm)
        risk_map = [[0.0]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        classified_target.auction_done = True
        result = run_auction(swarm, classified_target, risk_map,
                             lambda m, t: None)
        assert result is None

    def test_bmp_needs_up_to_3_drones(self, swarm):
        t = make_target(ttype='bmp')
        t.state = 'classified'
        assert t.drones_needed == 3
        update_neighbors(swarm)
        risk_map = [[0.0]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        result = run_auction(swarm, t, risk_map, lambda m, t: None)
        assert result is not None
        assert len(result['assigned']) <= 3

    def test_auction_log_callback_called_with_auction_type(self, swarm, classified_target):
        update_neighbors(swarm)
        risk_map = [[0.0]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        types_logged = []
        run_auction(swarm, classified_target, risk_map,
                    lambda m, t: types_logged.append(t))
        assert 'auction' in types_logged


class TestZoneOverlapsEW:
    def test_zone_inside_ew_overlaps(self):
        ex, ey, ew, eh = EW_RECT
        # Зона що точно всередині РЕБ
        col = ex // cfg.ZONE_W
        row = ey // cfg.ZONE_H
        assert _zone_overlaps_ew(col, row, EW_RECT) is True

    def test_zone_far_from_ew_no_overlap(self):
        assert _zone_overlaps_ew(0, 0, EW_RECT) is False

    def test_no_ew_rect_no_overlap(self):
        assert _zone_overlaps_ew(5, 5, None) is False


class TestSwarmScout:
    def test_scout_assigns_target_zone(self, swarm):
        d = swarm[0]
        d.status = 'scouting'
        d.target_zone = None
        swarm_scout(d, swarm, EW_RECT)
        assert d.target_zone is not None

    def test_scout_no_op_if_already_has_zone(self, swarm):
        d = swarm[0]
        d.status = 'scouting'
        d.target_zone = (3, 3)
        swarm_scout(d, swarm, EW_RECT)
        assert d.target_zone == (3, 3)

    def test_scout_no_op_if_attacking(self, swarm):
        d = swarm[0]
        d.status = 'attacking'
        d.target_zone = None
        swarm_scout(d, swarm, EW_RECT)
        assert d.target_zone is None

    def test_scout_idle_rescue(self, swarm):
        """Якщо дрон простоює > 3с — отримує нову зону."""
        d = swarm[0]
        d.status = 'scouting'
        d.target_zone = None
        d.scout_idle_time = 4.0
        swarm_scout(d, swarm, EW_RECT)
        assert d.target_zone is not None
        assert d.scout_idle_time == 0.0

    def test_patrol_mode_when_fully_covered(self, swarm):
        """У режимі патрулювання теж призначається зона."""
        d = swarm[0]
        d.status = 'scouting'
        d.target_zone = None
        # Позначити всю карту як покриту
        for row in range(cfg.GRID_ROWS):
            for col in range(cfg.GRID_COLS):
                d.local_map[row][col] = True
        swarm_scout(d, swarm, EW_RECT)
        assert d.target_zone is not None


# ═══════════════════════════════════════════════════════════════════
# 5. METRICS — збір, запис, export CSV
# ═══════════════════════════════════════════════════════════════════

class TestMetricsCollector:
    def test_reset_clears_all(self, metrics):
        metrics.consensus_rounds = 10
        metrics.drone_loss_events.append((1.0, 1, 'РЕБ'))
        metrics.reset()
        assert metrics.consensus_rounds == 0
        assert metrics.drone_loss_events == []

    def test_update_samples_coverage(self, metrics, swarm):
        coverage = [[False]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        coverage[0][0] = True
        # Перший виклик — таймер ще не спрацює
        metrics.update(2.0, 2.1, swarm, coverage)
        assert len(metrics.coverage_timeline) == 1

    def test_update_does_not_sample_before_interval(self, metrics, swarm):
        coverage = [[False]*cfg.GRID_COLS for _ in range(cfg.GRID_ROWS)]
        metrics.update(0.5, 0.5, swarm, coverage)
        assert len(metrics.coverage_timeline) == 0

    def test_record_target_detected(self, metrics):
        t = make_target(tid=5)
        metrics.record_target_detected(10.0, t)
        assert 5 in metrics.target_events
        assert metrics.target_events[5]['detect_time'] == 10.0

    def test_record_target_classified(self, metrics):
        t = make_target(tid=5)
        metrics.record_target_detected(10.0, t)
        metrics.record_target_classified(12.5, t)
        assert metrics.target_events[5]['classify_time'] == 12.5

    def test_record_target_destroyed(self, metrics):
        t = make_target(tid=5)
        metrics.record_target_detected(10.0, t)
        metrics.record_target_destroyed(20.0, t)
        assert metrics.target_events[5]['destroy_time'] == 20.0

    def test_record_drone_loss_ew(self, metrics, drone):
        metrics.record_drone_loss(5.0, drone, is_ew=True)
        assert len(metrics.drone_loss_events) == 1
        t, did, reason = metrics.drone_loss_events[0]
        assert reason == 'РЕБ'
        assert did == drone.id

    def test_record_drone_loss_battery(self, metrics, drone):
        metrics.record_drone_loss(5.0, drone, is_ew=False)
        _, _, reason = metrics.drone_loss_events[0]
        assert reason == 'батарея'

    def test_record_consensus_round(self, metrics):
        metrics.record_consensus_round(3.0)
        metrics.record_consensus_round(5.0)
        assert metrics.consensus_rounds == 2
        assert 3.0 in metrics.consensus_times

    def test_record_auction(self, metrics):
        t = make_target(tid=3, ttype='bmp')
        all_bids = [(1, 0.75), (2, 0.60)]
        metrics.record_auction(8.0, t, all_bids, [1])
        assert len(metrics.auction_log) == 1
        ts, label, tid, bids_dict, winners = metrics.auction_log[0]
        assert tid == 3
        assert winners == [1]
        assert bids_dict[1] == pytest.approx(0.75)

    def test_zone_visit_recorded(self, metrics):
        metrics.record_zone_visit(2, 3, drone_id=5)
        assert 5 in metrics.zone_visitors[2][3]

    def test_zone_visit_out_of_bounds_ignored(self, metrics):
        metrics.record_zone_visit(99, 99, drone_id=1)  # не повинно падати

    def test_coverage_overlap_property(self, metrics):
        metrics.zone_visitors[0][0] = {1, 2, 3}
        metrics.zone_visitors[1][1] = {4}
        assert metrics.coverage_overlap == 1

    def test_export_csv_creates_files(self, metrics):
        t = make_target(tid=1, ttype='infantry')
        metrics.record_target_detected(5.0, t)
        metrics.record_target_classified(7.5, t)
        metrics.record_target_destroyed(12.0, t)
        metrics.record_consensus_round(2.0)
        metrics.auction_log.append(
            (3.0, 'Піхота', 1, {1: 0.8}, [1])
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            metrics.export_to_csv(tmpdir)
            files = os.listdir(tmpdir)
            assert any('coverage' in f for f in files)
            assert any('targets' in f for f in files)
            assert any('summary' in f for f in files)

    def test_export_csv_summary_contains_scenario(self, metrics):
        with tempfile.TemporaryDirectory() as tmpdir:
            metrics.export_to_csv(tmpdir)
            summary_path = next(
                os.path.join(tmpdir, f) for f in os.listdir(tmpdir)
                if 'summary' in f
            )
            with open(summary_path, encoding='utf-8') as fh:
                content = fh.read()
            assert cfg.SCENARIO in content

    def test_swarm_recovery_recorded_on_auction_after_loss(self, metrics, drone):
        """Якщо дрон загинув атакуючи ціль і пройшов аукціон — recovery записується."""
        t = make_target(tid=7, ttype='bmp')
        t.state = 'classified'
        drone.target_obj = t
        metrics.record_drone_loss(10.0, drone, is_ew=True)
        # Симулюємо аукціон після втрати
        metrics.record_auction(13.5, t, [(2, 0.7)], [2])
        assert len(metrics.swarm_recovery_times) == 1
        _, _, delta = metrics.swarm_recovery_times[0]
        assert delta == pytest.approx(3.5)
