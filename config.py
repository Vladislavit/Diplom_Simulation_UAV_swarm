# config.py — Конфігурація та константи симуляції рою БПЛА

# === Розміри вікна ===
WINDOW_WIDTH = 1792    # 1280 × 1.4
WINDOW_HEIGHT = 1008   # 720  × 1.4
MAP_WIDTH = 1452       # WINDOW_WIDTH − PANEL_WIDTH
PANEL_WIDTH = 340
FPS = 60

# === Сітка зон покриття ===
GRID_COLS = 15         # 1452 ÷ 96 ≈ 15  →  ZONE_W = 96 px (≈ 94 старих)
GRID_ROWS = 14         # 1008 ÷ 72 = 14  →  ZONE_H = 72 px (незмінний)
ZONE_W = MAP_WIDTH // GRID_COLS   # 96 пікселів
ZONE_H = WINDOW_HEIGHT // GRID_ROWS  # 72 пікселі

# === Кольори карти ===
COLOR_FIELD = (45, 90, 45)
COLOR_FOREST = (25, 62, 25)
COLOR_ROAD = (175, 165, 145)
COLOR_BUILDING = (140, 135, 130)
COLOR_BUILDING_EDGE = (110, 105, 100)
COLOR_RIVER = (55, 105, 175)

# === Кольори інтерфейсу ===
COLOR_BG = (20, 22, 28)
COLOR_PANEL_BG = (28, 30, 40)
COLOR_PANEL_BORDER = (50, 55, 70)
COLOR_TEXT = (220, 220, 225)
COLOR_TEXT_DIM = (140, 140, 150)
COLOR_TEXT_BRIGHT = (255, 255, 255)

# === Кольори зон ===
COLOR_ZONE_COVERED = (0, 200, 50, 55)
COLOR_ZONE_ACTIVE = (0, 200, 50, 25)
COLOR_ZONE_GRID = (255, 255, 255, 18)

# === РЕБ зона ===
COLOR_EW_FILL = (220, 30, 30, 35)
COLOR_EW_BORDER = (220, 50, 50)
EW_W = 294          # ширина РЕБ зони (пікселі): 210 × 1.4
EW_H = 266          # висота РЕБ зони (пікселі): 190 × 1.4
# Позиція (x, y) генерується в reset() з MAP_SEED → зберігається в sim.ew_rect
EW_LOSS_TIME_MIN = 1.5
EW_LOSS_TIME_MAX = 3.5
EW_LOSS_PROB_PER_SEC = 0.35  # 35% шанс втрати за кожну секунду в зоні

# === Кольори дронів ===
COLOR_DRONE_SCOUT = (60, 140, 255)
COLOR_DRONE_ATTACK = (255, 160, 40)
COLOR_DRONE_LOW_BAT = (255, 230, 50)
COLOR_DRONE_LOST = (130, 130, 130)

# === Кольори логу ===
COLOR_LOG_CONSENSUS = (100, 185, 255)
COLOR_LOG_AUCTION = (100, 230, 110)
COLOR_LOG_THREAT = (255, 85, 85)
COLOR_LOG_WARNING = (255, 225, 85)
COLOR_LOG_INFO = (180, 180, 190)

# === Параметри дронів ===
NUM_DRONES = 15
CLUSTER_SIZE = 3            # 15/3 = 5 статичних кластерів (hybrid)
DRONE_SPEED = 15.0          # ~80 км/г у масштабі карти
BATTERY_DRAIN_RATE = 0.35   # відсотків на секунду
DETECTION_RADIUS = 65       # пікселі
COMM_RADIUS = 180           # пікселі
SECTOR_BONUS = 200.0                    # бонус за зону в своєму секторі
LOW_BATTERY = 25            # поріг низького заряду (%)
INERTIA = 0.88              # коефіцієнт інерції руху (0..1)

# === Параметри цілей ===
NUM_TARGETS = 12
STAGNATION_TIME = 100.0  # секунд без прогресу → завершення місії
TARGET_TYPES = {
    'infantry': {'min_drones': 1, 'max_drones': 1, 'label': 'Піхота'},
    'vehicle':  {'min_drones': 1, 'max_drones': 2, 'label': 'Авто'},
    'bmp':      {'min_drones': 2, 'max_drones': 3, 'label': 'БМП'},
    'tank':     {'min_drones': 3, 'max_drones': 3, 'label': 'Танк'},
}
# Імовірність ураження за один камікадзе-удар (залежить від типу цілі).
# Заміняє фіксований drones_needed: дрони шлються по одному, очікувана
# к-сть на знищення = 1/p (піхота ≈1.1, авто ≈1.8, БМП ≈2.6, танк ≈5.6).
HIT_PROB = {
    'infantry': 0.90,
    'vehicle':  0.55,
    'bmp':      0.38,
    'tank':     0.18,
}
CLASSIFY_TIME = 2.5    # секунд для класифікації після виявлення
DESTROY_DISTANCE = 15  # відстань для знищення цілі (пікселі)

# === Ваги аукціонного алгоритму ===
W_DISTANCE = 0.30       # зворотна відстань
W_BATTERY = 0.25        # заряд батареї
W_CONNECTIVITY = 0.20   # зв'язок із сусідами
W_OBSTACLES = 0.15      # перешкоди
W_RISK = 0.10           # ризик зони
W_TANK_PENALTY = 0.15   # множник bid для танку (висока небезпека)
MIN_BID_THRESHOLD = 0.25        # мінімальний bid для призначення
FALLBACK_SCOUT_TIME = 20.0      # секунд простою → fallback-призначення

# === Консенсусний алгоритм ===
CONSENSUS_INTERVAL = 2.0           # intra-cluster обмін (секунди)
INTER_CLUSTER_INTERVAL = CONSENSUS_INTERVAL * 3  # inter-cluster обмін рідше

# === Boids (swarm_only) ===
BOIDS_SEP_RADIUS = 80       # радіус відштовхування (px)
BOIDS_SEP_WEIGHT = 3.0      # вага separation (домінує над cohesion)
BOIDS_ALI_WEIGHT = 0.8      # вага alignment
BOIDS_COH_WEIGHT = 0.1      # вага cohesion (мінімальна — не злипатись)
BOIDS_WANDER = 0.8          # випадковий імпульс блукання (оновл. кожні 2-3с)

# === Leader-Follower ===
FOLLOWER_DIST = 120         # комфортна відстань від лідера (px)

# === Сценарій симуляції ===
# 'greedy'          — кожен дрон самостійно атакує найближчу відому ціль
# 'swarm_only'      — тільки ройовий інтелект, атак немає
# 'auction'         — аукціон з bid, без РЕБ-адаптації і консенсусу
# 'hybrid'          — аукціон + консенсус + РЕБ-адаптація (повна система)
# 'leader_follower' — ієрархія: 1 глобальний лідер (без переобрання)
#                     керує follower-дронами; гине лідер — рій деградує
SCENARIO = 'hybrid'

# === Генерація карти ===
MAP_SEED = 42

# === Стартові позиції дронів (формація біля бази) ===
DRONE_START_X = 80
DRONE_START_Y = 854   # 610 × 1008/720 — зберігає позицію біля нижнього краю
DRONE_SPACING_X = 85
DRONE_SPACING_Y = 50
DRONE_COLS = 5
