from ursina import *
from enum import Enum, auto
import heapq
from math import degrees, atan2
import json
import urllib.request

app = Ursina()

# --- CONFIGURACIÓN ---
size = 5
half = size / 2

# ── Texturas procedurales (no requieren archivos externos) ──────────────────
from PIL import Image, ImageDraw

def _gen_tex(bg_rgba, line_rgba, grid_size=5, res=512):
    img = Image.new("RGBA", (res, res), bg_rgba)
    draw = ImageDraw.Draw(img)
    lw = 6
    for i in range(grid_size + 1):
        pos = int((i / grid_size) * (res - lw / 2))
        draw.line((0, pos, res, pos), fill=line_rgba, width=lw)
        draw.line((pos, 0, pos, res), fill=line_rgba, width=lw)
    return Texture(img)

wall_tex  = _gen_tex((130,155,185,255), (210,220,235,200), grid_size=5)
floor_tex = _gen_tex((35,38,48,255),   (220,220,235,200), grid_size=5)

# ── Temporizador ──────────────────────────────────────────────
TIEMPO_LIMITE = 90
tiempo_restante = TIEMPO_LIMITE
juego_activo    = True
pedidos_completados = 0
score = 0

# ──────────────────────────────────────────────────────────────
#  PATHFINDING A* + DANGER MAP (prefiere rutas alejadas de paredes)
# ──────────────────────────────────────────────────────────────
GRID_RES  = 0.25
GRID_COLS = int(size / GRID_RES)   # 20
GRID_ROWS = int(size / GRID_RES)   # 20

# Radio del bot: escala 0.5 → radio real 0.25, más margen mínimo
BOT_RADIUS = 0.26

WALL_MARGIN   = 0.30
# Pared gris: scale_x=0.1 → half-extent=0.05. Con radio del bot + margen de seguridad:
GRAY_X_MARGIN = 0.05 + BOT_RADIUS + 0.05   # ≈ 0.36
gray_wall_center_x = 0.0
GRAY_Z_MIN, GRAY_Z_MAX = -2.5, 0.0

# Parámetros del danger map
DANGER_RADIUS = 3      # celdas de "zona de miedo" alrededor de obstáculos
DANGER_MAX    = 4.0    # coste extra máximo (celda pegada a pared)

def _cell_center(row, col):
    return (-half + (col + 0.5) * GRID_RES,
            -half + (row + 0.5) * GRID_RES)

def _build_grids(station_positions=None):
    from collections import deque

    blocked = [[False]*GRID_COLS for _ in range(GRID_ROWS)]
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            wx, wz = _cell_center(r, c)
            if (wx < -half + WALL_MARGIN or wx > half - WALL_MARGIN or
                    wz < -half + WALL_MARGIN or wz > half - WALL_MARGIN):
                blocked[r][c] = True
            elif abs(wx - gray_wall_center_x) < GRAY_X_MARGIN and GRAY_Z_MIN <= wz <= GRAY_Z_MAX:
                blocked[r][c] = True

    # Bloquear celdas ocupadas por estaciones (scale=0.7 → half=0.35)
    if station_positions:
        st_half = STATION_SCALE * 0.5 + BOT_RADIUS
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                wx, wz = _cell_center(r, c)
                for sp in station_positions:
                    if abs(wx - sp.x) < st_half and abs(wz - sp.z) < st_half:
                        blocked[r][c] = True
                        break

    # BFS desde frontera de celdas bloqueadas → mapa de distancia
    dist   = [[999]*GRID_COLS for _ in range(GRID_ROWS)]
    danger = [[0.0]*GRID_COLS for _ in range(GRID_ROWS)]
    queue  = deque()

    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            if blocked[r][c]:
                dist[r][c] = 0
                for dr, dc in ((-1,0),(1,0),(0,-1),(0,1)):
                    nr, nc = r+dr, c+dc
                    if 0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS:
                        if not blocked[nr][nc] and dist[nr][nc] == 999:
                            dist[nr][nc] = 1
                            queue.append((nr, nc))

    while queue:
        r, c = queue.popleft()
        d = dist[r][c]
        if d >= DANGER_RADIUS:
            continue
        for dr, dc in ((-1,0),(1,0),(0,-1),(0,1),
                       (-1,-1),(-1,1),(1,-1),(1,1)):
            nr, nc = r+dr, c+dc
            if 0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS:
                if not blocked[nr][nc] and dist[nr][nc] == 999:
                    dist[nr][nc] = d + 1
                    queue.append((nr, nc))

    # Distancia → coste extra (cerca = caro)
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            d = dist[r][c]
            if d < DANGER_RADIUS:
                t = d / DANGER_RADIUS          # 0 (pegado) → 1 (lejos)
                danger[r][c] = DANGER_MAX * (1.0 - t)

    return blocked, danger

NAV_GRID, DANGER_GRID = None, None  # se calcula después de crear estaciones

def world_to_cell(wx, wz):
    col = int((wx + half) / GRID_RES)
    row = int((wz + half) / GRID_RES)
    return (max(0, min(GRID_ROWS-1, row)),
            max(0, min(GRID_COLS-1, col)))

def cell_to_world(row, col):
    return _cell_center(row, col)

def nearest_free_cell(goal):
    if not NAV_GRID[goal[0]][goal[1]]:
        return goal
    best, best_d = None, 9999
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            if not NAV_GRID[r][c]:
                d = abs(r-goal[0]) + abs(c-goal[1])
                if d < best_d:
                    best_d, best = d, (r, c)
    return best

def heuristic(a, b):
    dr, dc = abs(a[0]-b[0]), abs(a[1]-b[1])
    return max(dr, dc) + (1.414-1)*min(dr, dc)

def astar(start_world, goal_world, extra_blocked=None):
    """A* con danger map: penaliza rutas cercanas a paredes y obstáculos.
    extra_blocked: set de (row, col) con celdas temporalmente bloqueadas (otros bots)."""
    start = world_to_cell(start_world.x, start_world.z)
    goal  = nearest_free_cell(world_to_cell(goal_world.x, goal_world.z))
    if goal is None:
        return []
    if start == goal:
        return [goal_world]

    open_heap = []
    counter   = 0
    heapq.heappush(open_heap, (heuristic(start, goal), counter, start))
    came_from = {start: None}
    g_score   = {start: 0.0}

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            path, node = [], goal
            while node is not None:
                wx, wz = cell_to_world(node[0], node[1])
                path.append(Vec3(wx, goal_world.y, wz))
                node = came_from[node]
            path.reverse()
            return path

        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = current[0]+dr, current[1]+dc
                if not (0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS):
                    continue
                if NAV_GRID[nr][nc]:
                    continue
                if extra_blocked and (nr, nc) in extra_blocked:
                    continue
                if dr != 0 and dc != 0:
                    if NAV_GRID[current[0]+dr][current[1]] or \
                       NAV_GRID[current[0]][current[1]+dc]:
                        continue
                step = 1.414 if (dr != 0 and dc != 0) else 1.0
                ng   = g_score[current] + step + DANGER_GRID[nr][nc]
                neighbor = (nr, nc)
                if ng < g_score.get(neighbor, 1e9):
                    g_score[neighbor]   = ng
                    came_from[neighbor] = current
                    counter += 1
                    heapq.heappush(open_heap,
                        (ng + heuristic(neighbor, goal), counter, neighbor))
    return []


# ──────────────────────────────────────────────────────────────
#  COLISIONES: lista de obstáculos estáticos (cajas 1x1x1)
# ──────────────────────────────────────────────────────────────
static_obstacles = []   # lista de Vec3 (centro XZ de cada estación)
all_bots = []           # lista de todas las instancias BotBase (se llena al crear cada bot)

def register_obstacle(pos):
    static_obstacles.append(Vec3(pos.x, pos.z, 0))  # guardamos x,z

# ──────────────────────────────────────────────────────────────
#  SEPARACIÓN ENTRE BOTS  (steering suave)
# ──────────────────────────────────────────────────────────────
BOT_SEP_RADIUS = 0.55    # distancia mínima entre centros de bots
BOT_SEP_FORCE  = 6.0     # fuerza de empuje

def apply_bot_separation(bot, dt):
    """Empuja al bot para que no se solape con otros bots."""
    px, pz = bot.e.position.x, bot.e.position.z
    push_x, push_z = 0.0, 0.0

    for other in all_bots:
        if other is bot:
            continue
        ox, oz = other.e.position.x, other.e.position.z
        dx, dz = px - ox, pz - oz
        dist = (dx * dx + dz * dz) ** 0.5
        if dist < BOT_SEP_RADIUS and dist > 0.001:
            # Fuerza inversamente proporcional a la distancia
            overlap = (BOT_SEP_RADIUS - dist) / BOT_SEP_RADIUS
            push_x += (dx / dist) * overlap * BOT_SEP_FORCE * dt
            push_z += (dz / dist) * overlap * BOT_SEP_FORCE * dt

    if push_x != 0.0 or push_z != 0.0:
        new_pos = Vec3(px + push_x, bot.e.position.y, pz + push_z)
        new_pos = push_out_of_obstacles(new_pos)
        new_pos = push_out_of_gray_wall(new_pos)
        bot.e.position = new_pos

def push_out_of_obstacles(bot_pos):
    """Empuja la posición del bot para que no se solape con estaciones 1x1."""
    px, pz = bot_pos.x, bot_pos.z
    for obs in static_obstacles:
        ox, oz = obs.x, obs.y
        # Caja AABB expandida con el radio del bot
        expand = 0.5 + BOT_RADIUS
        dx = px - ox
        dz = pz - oz
        if abs(dx) < expand and abs(dz) < expand:
            pen_x = expand - abs(dx)
            pen_z = expand - abs(dz)
            # Empujar por el eje de MENOR penetración
            if pen_x <= pen_z:
                px += pen_x * (1 if dx >= 0 else -1)
            else:
                pz += pen_z * (1 if dz >= 0 else -1)
    # Confinar dentro de los muros de ladrillo
    px = max(-half + WALL_MARGIN, min(half - WALL_MARGIN, px))
    pz = max(-half + WALL_MARGIN, min(half - WALL_MARGIN, pz))
    return Vec3(px, bot_pos.y, pz)

def push_out_of_gray_wall(bot_pos):
    """Evitar que el bot atraviese la pared gris interna."""
    px, pz = bot_pos.x, bot_pos.z
    expand_x = 0.05 + BOT_RADIUS
    if abs(px - gray_wall_center_x) < expand_x and GRAY_Z_MIN - BOT_RADIUS < pz < GRAY_Z_MAX + BOT_RADIUS:
        px = gray_wall_center_x + expand_x * (1 if px >= gray_wall_center_x else -1)
    return Vec3(px, bot_pos.y, pz)

# ──────────────────────────────────────────────────────────────
#  PAREDES / PISO
# ──────────────────────────────────────────────────────────────
faces_data = [
    {'pos': (0, 0,  half), 'rot': (0, 180, 0), 'eje': 'z', 'dir':  1, 'tex': wall_tex},
    {'pos': (0, 0, -half), 'rot': (0, 0,   0), 'eje': 'z', 'dir': -1, 'tex': wall_tex},
    {'pos': ( half, 0, 0), 'rot': (0,  90, 0), 'eje': 'x', 'dir':  1, 'tex': wall_tex},
    {'pos': (-half, 0, 0), 'rot': (0, -90, 0), 'eje': 'x', 'dir': -1, 'tex': wall_tex},
    {'pos': (0, -half, 0), 'rot': (-90, 0, 0), 'eje': 'y', 'dir': -1, 'tex': floor_tex},
]

walls = []
for face in faces_data:
    w = Entity(model='quad', scale=(size, size),
               position=face['pos'], rotation=face['rot'],
               texture=face['tex'], double_sided=True)
    w.texture_scale = (size, size)
    w.eje, w.dir = face['eje'], face['dir']
    walls.append(w)

# ──────────────────────────────────────────────────────────────
#  HELPER: CUBO ESTILO DIBUJO
# ──────────────────────────────────────────────────────────────
STATION_SCALE = 0.7
STATION_Y = -half + STATION_SCALE * 0.5

def Station(pos, col):
    e = Entity(model='cube', color=col, position=pos, scale=STATION_SCALE,
               unlit=True, edge_color=color.black, edge_width=2)
    register_obstacle(pos)
    return e

# ──────────────────────────────────────────────────────────────
#  ESTACIONES (posiciones fijas, más pequeñas, tocando el suelo)
# ──────────────────────────────────────────────────────────────
pos_tomate     = Vec3( 1, STATION_Y, -2)
pos_lechuga    = Vec3( 2, STATION_Y, -2)
pos_corte      = Vec3(-2, STATION_Y, -2)
pos_ensamblaje = Vec3(-2, STATION_Y, -1)
pos_platos     = Vec3(-2, STATION_Y,  0)
pos_entrega    = Vec3( 2, STATION_Y,  1)

st_tomate     = Station(pos_tomate,     color.red)
st_lechuga    = Station(pos_lechuga,    color.green)
st_corte      = Station(pos_corte,      color.yellow)
st_ensamblaje = Station(pos_ensamblaje, color.brown)
st_platos     = Station(pos_platos,     color.white)
st_entrega    = Station(pos_entrega,    color.azure)

# Construir navgrid AHORA que las estaciones existen
station_positions = [pos_tomate, pos_lechuga, pos_corte, pos_ensamblaje, pos_platos, pos_entrega]
NAV_GRID, DANGER_GRID = _build_grids(station_positions)

# ── Posiciones de ACCESO (frente a cada estación, lado interior) ──
# Los bots navegan hasta aquí en lugar de al centro de la caja.
_BOT_Y = -half + 0.25   # base del bot toca el suelo (scale=0.5)
acc_tomate     = Vec3( 1.0, _BOT_Y, -1.3)
acc_lechuga    = Vec3( 2.0, _BOT_Y, -1.3)
acc_corte      = Vec3(-1.3, _BOT_Y, -2.0)
acc_ensamblaje = Vec3(-1.3, _BOT_Y, -1.0)
acc_platos     = Vec3(-1.3, _BOT_Y,  0.0)
acc_entrega    = Vec3( 1.3, _BOT_Y,  1.0)

# ── Posiciones de espera idle – fuera de las rutas de B1/B2 ──
pos_idle_b2 = Vec3( 0.0, _BOT_Y,  2.0)
pos_idle_b3 = Vec3( 1.5, _BOT_Y,  1.5)
pos_idle_b4 = Vec3( 2.0, _BOT_Y,  0.5)

# Pared interna gris (obstáculo)
gray_wall = Entity(model='cube', color=color.gray,
                   position=(0, -half + 2.5, -1.25),
                   scale=(0.1, 5, 2.5), unlit=True)

# ──────────────────────────────────────────────────────────────
#  INGREDIENTE VISUAL
# ──────────────────────────────────────────────────────────────
def crear_ingrediente(pos, col, nombre):
    e = Entity(model='sphere', color=col, position=pos, scale=0.35, unlit=True)
    e.nombre = nombre
    all_ingredientes.append(e)
    return e

# ──────────────────────────────────────────────────────────────
#  ESTADOS DE CADA BOT
# ──────────────────────────────────────────────────────────────
class EstadoBot1(Enum):
    IR_TOMATE   = auto()
    IR_CORTE_T  = auto()
    IR_LECHUGA  = auto()
    IR_CORTE_L  = auto()

class EstadoBot2(Enum):
    ESPERAR  = auto()
    IR_IDLE  = auto()
    IR_CORTE = auto()
    CORTAR   = auto()
    IR_ENSAM = auto()

class EstadoBot3(Enum):
    ESPERAR  = auto()
    IR_IDLE  = auto()
    IR_ENSAM = auto()
    IR_PLATO = auto()

class EstadoBot4(Enum):
    ESPERAR    = auto()
    IR_IDLE    = auto()
    IR_PLATO   = auto()
    IR_ENTREGA = auto()

# ──────────────────────────────────────────────────────────────
#  COLAS COMPARTIDAS
# ──────────────────────────────────────────────────────────────
cola_corte      = []
cola_ensamblaje = []
cola_platos     = []
plato_actual    = []
all_ingredientes = []

# ──────────────────────────────────────────────────────────────
#  CLASE BASE CON PATHFINDING
# ──────────────────────────────────────────────────────────────
class BotBase:
    SPEED     = 3.0
    REACH     = 0.35   # distancia para considerar "llegó al waypoint"

    def __init__(self):
        self.waypoints = []   # lista de Vec3 generada por A*
        self._destino_final = None
        self._last_pos = None
        self._stuck_t = 0.0
        all_bots.append(self)

    def _blocked_by_bots(self):
        """Retorna set de celdas ocupadas por otros bots."""
        blocked = set()
        for bot in all_bots:
            if bot is self:
                continue
            r, c = world_to_cell(bot.e.position.x, bot.e.position.z)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS:
                        blocked.add((nr, nc))
        return blocked

    def set_destino(self, target: Vec3, force=False):
        """Calcula ruta A* hacia target, evitando otros bots."""
        if not force and target == self._destino_final:
            return
        self._destino_final = target
        blocked = self._blocked_by_bots()
        path = astar(self.e.position, target, extra_blocked=blocked)
        if path:
            self.waypoints = path
        else:
            self.waypoints = [target]

    def mover(self, dt):
        """Avanza por los waypoints. Devuelve True si aún está en movimiento."""
        if not self.waypoints:
            return False

        wp = self.waypoints[0]
        dir_vec = wp - self.e.position
        dir_vec.y = 0
        dist = dir_vec.length()

        if dist > self.REACH:
            move = dir_vec.normalized() * self.SPEED * dt
            new_pos = self.e.position + move

            # Rotación suave hacia la dirección de movimiento
            target_rot = degrees(atan2(dir_vec.x, dir_vec.z))
            diff = (target_rot - self.e.rotation_y + 180) % 360 - 180
            self.e.rotation_y += diff * 10 * dt

            # Colisiones con obstáculos y paredes
            new_pos = push_out_of_obstacles(new_pos)
            new_pos = push_out_of_gray_wall(new_pos)

            self.e.position = new_pos

            # Separación con otros bots
            apply_bot_separation(self, dt)

            # Detectar atasco
            if self._last_pos is None:
                self._last_pos = self.e.position
            d_moved = (self.e.position - self._last_pos).length()
            if d_moved < 0.02:
                self._stuck_t += dt
            else:
                self._stuck_t = 0.0
                self._last_pos = self.e.position

            # Recalcular si está atascado > 0.15s (más rápido = menos bloqueos)
            if self._stuck_t > 0.15 and self._destino_final:
                self._stuck_t = 0.0
                self.set_destino(self._destino_final, force=True)

            # Llevar la carga encima
            if hasattr(self, 'carga') and self.carga:
                self.carga.position = self.e.position + Vec3(0, 0.5, 0)
            if hasattr(self, 'cargas'):
                for idx, c in enumerate(self.cargas):
                    c.position = self.e.position + Vec3((idx - 0.5) * 0.3, 0.6, 0)
            return True
        else:
            # Waypoint alcanzado
            self.waypoints.pop(0)
            self._stuck_t = 0.0
            if not self.waypoints:
                dest = self._destino_final
                self.e.position = Vec3(dest.x, self.e.position.y, dest.z)
                if hasattr(self, 'carga') and self.carga:
                    self.carga.position = self.e.position + Vec3(0, 0.5, 0)
                apply_bot_separation(self, dt)
                return False
            return True

    @property
    def llegó(self):
        return not self.waypoints and self._destino_final is not None

# ──────────────────────────────────────────────────────────────
#  BOT 1 – Recolector
# ──────────────────────────────────────────────────────────────
class Bot1(BotBase):
    SPEED = 3.5

    def __init__(self):
        super().__init__()
        self.e = Entity(model='cube', color=color.orange,
                        position=Vec3(0, _BOT_Y, 2),
                        scale=0.5, unlit=True, edge_color=color.black, edge_width=2)
        self.label = Text(text='B1', world_parent=self.e,
                          position=(0, 0.7, 0), scale=6,
                          billboard=True, color=color.white)
        self.estado = EstadoBot1.IR_TOMATE
        self.carga  = None
        self.timer  = 0.0
        self.set_destino(acc_tomate)

    def update(self, dt):
        if not juego_activo:
            return

        en_movimiento = self.mover(dt)

        if en_movimiento:
            return

        # ── Acciones al llegar ────────────────────────────────
        if self.estado == EstadoBot1.IR_TOMATE:
            self.timer += dt
            if self.timer >= 0.5:
                self.timer = 0
                self.carga = crear_ingrediente(self.e.position + Vec3(0, 0.5, 0),
                                               color.red, 'tomate')
                self.estado = EstadoBot1.IR_CORTE_T
                self._destino_final = None
                self.set_destino(acc_corte)

        elif self.estado == EstadoBot1.IR_CORTE_T:
            cola_corte.append(self.carga)
            self.carga.position = pos_corte + Vec3(0, 0.8 + len(cola_corte) * 0.4, 0)
            self.carga  = None
            self.estado = EstadoBot1.IR_LECHUGA
            self._destino_final = None
            self.set_destino(acc_lechuga)

        elif self.estado == EstadoBot1.IR_LECHUGA:
            self.timer += dt
            if self.timer >= 0.5:
                self.timer = 0
                self.carga = crear_ingrediente(self.e.position + Vec3(0, 0.5, 0),
                                               color.lime, 'lechuga')
                self.estado = EstadoBot1.IR_CORTE_L
                self._destino_final = None
                self.set_destino(acc_corte)

        elif self.estado == EstadoBot1.IR_CORTE_L:
            cola_corte.append(self.carga)
            self.carga.position = pos_corte + Vec3(0, 0.8 + len(cola_corte) * 0.4, 0)
            self.carga  = None
            self.estado = EstadoBot1.IR_TOMATE
            self._destino_final = None
            self.set_destino(acc_tomate)

    def get_status(self):
        return {
            EstadoBot1.IR_TOMATE:  "Recolectando tomate",
            EstadoBot1.IR_CORTE_T: "Llevando tomate",
            EstadoBot1.IR_LECHUGA: "Recolectando lechuga",
            EstadoBot1.IR_CORTE_L: "Llevando lechuga",
        }.get(self.estado, "Desconocido")


# ──────────────────────────────────────────────────────────────
#  BOT 2 – Cortador
# ──────────────────────────────────────────────────────────────
class Bot2(BotBase):
    SPEED    = 3.0
    T_CORTAR = 1.2

    def __init__(self):
        super().__init__()
        self.e = Entity(model='cube', color=color.magenta,
                        position=Vec3(0, _BOT_Y, 1.5),
                        scale=0.5, unlit=True, edge_color=color.black, edge_width=2)
        self.label = Text(text='B2', world_parent=self.e,
                          position=(0, 0.7, 0), scale=6,
                          billboard=True, color=color.white)
        self.estado = EstadoBot2.ESPERAR
        self.carga  = None
        self.timer  = 0.0

    def update(self, dt):
        if not juego_activo:
            return

        if self.estado == EstadoBot2.ESPERAR:
            if cola_corte:
                self.estado = EstadoBot2.IR_CORTE
                self._destino_final = None
                self.set_destino(acc_corte)
            else:
                self.estado = EstadoBot2.IR_IDLE
                self._destino_final = None
                self.set_destino(pos_idle_b2)
            return

        if self.estado == EstadoBot2.IR_IDLE:
            if cola_corte:
                self.estado = EstadoBot2.IR_CORTE
                self._destino_final = None
                self.set_destino(acc_corte)
            else:
                en_movimiento = self.mover(dt)
                if not en_movimiento:
                    apply_bot_separation(self, dt)
            return

        if self.estado == EstadoBot2.CORTAR:
            self.timer += dt
            pulso = 1 + 0.15 * sin(self.timer * 10)
            if self.carga:
                self.carga.scale = pulso * 0.35
                self.carga.position = self.e.position + Vec3(0, 0.5, 0)
            if self.timer >= self.T_CORTAR:
                if self.carga:
                    self.carga.scale_y = 0.15
                    self.carga.scale_x = 0.45
                    self.carga.scale_z = 0.45
                    self.carga.color   = lerp(self.carga.color, color.white, 0.4)
                    self.carga.nombre += '_cortado'
                self.estado = EstadoBot2.IR_ENSAM
                self._destino_final = None
                self.set_destino(acc_ensamblaje)
            return

        en_movimiento = self.mover(dt)
        if en_movimiento:
            return

        if self.estado == EstadoBot2.IR_CORTE:
            if cola_corte:
                self.carga  = cola_corte.pop(0)
                self.estado = EstadoBot2.CORTAR
                self.timer  = 0
            else:
                self.estado = EstadoBot2.ESPERAR

        elif self.estado == EstadoBot2.IR_ENSAM:
            cola_ensamblaje.append(self.carga)
            self.carga.position = pos_ensamblaje + Vec3(0, 0.8 + len(cola_ensamblaje) * 0.3, 0)
            self.carga  = None
            self.estado = EstadoBot2.ESPERAR

    def get_status(self):
        return {
            EstadoBot2.ESPERAR: "Esperando",
            EstadoBot2.IR_IDLE: "Idle",
            EstadoBot2.IR_CORTE: "Yendo a cortar",
            EstadoBot2.CORTAR: "Cortando...",
            EstadoBot2.IR_ENSAM: "Llevando a ensamblaje",
        }.get(self.estado, "Desconocido")


# ──────────────────────────────────────────────────────────────
#  BOT 3 – Ensamblador
# ──────────────────────────────────────────────────────────────
class Bot3(BotBase):
    SPEED = 2.8

    def __init__(self):
        super().__init__()
        self.e = Entity(model='cube', color=color.cyan,
                        position=Vec3(1, _BOT_Y, 0),
                        scale=0.5, unlit=True, edge_color=color.black, edge_width=2)
        self.label = Text(text='B3', world_parent=self.e,
                          position=(0, 0.7, 0), scale=6,
                          billboard=True, color=color.white)
        self.estado = EstadoBot3.ESPERAR
        self.carga  = None

    def update(self, dt):
        if not juego_activo:
            return

        if self.estado == EstadoBot3.ESPERAR:
            if cola_ensamblaje:
                self.estado = EstadoBot3.IR_ENSAM
                self._destino_final = None
                self.set_destino(acc_ensamblaje)
            else:
                self.estado = EstadoBot3.IR_IDLE
                self._destino_final = None
                self.set_destino(pos_idle_b3)
            return

        if self.estado == EstadoBot3.IR_IDLE:
            if cola_ensamblaje:
                self.estado = EstadoBot3.IR_ENSAM
                self._destino_final = None
                self.set_destino(acc_ensamblaje)
            else:
                en_movimiento = self.mover(dt)
                if not en_movimiento:
                    apply_bot_separation(self, dt)
            return

        en_movimiento = self.mover(dt)
        if en_movimiento:
            return

        if self.estado == EstadoBot3.IR_ENSAM:
            if cola_ensamblaje:
                self.carga  = cola_ensamblaje.pop(0)
                self.estado = EstadoBot3.IR_PLATO
                self._destino_final = None
                self.set_destino(acc_platos)
            else:
                self.estado = EstadoBot3.ESPERAR

        elif self.estado == EstadoBot3.IR_PLATO:
            plato_actual.append(self.carga)
            offset = (len(plato_actual) - 1) * 0.25
            self.carga.position = pos_platos + Vec3(offset - 0.12, 0.8, 0)
            self.carga = None
            nombres = [i.nombre for i in plato_actual]
            if any('tomate_cortado' in n for n in nombres) and \
               any('lechuga_cortado' in n for n in nombres):
                for ingrediente in plato_actual[:]:
                    cola_platos.append(ingrediente)
                plato_actual.clear()
            self.estado = EstadoBot3.ESPERAR

    def get_status(self):
        return {
            EstadoBot3.ESPERAR: "Esperando",
            EstadoBot3.IR_IDLE: "Idle",
            EstadoBot3.IR_ENSAM: "Yendo a ensamblaje",
            EstadoBot3.IR_PLATO: "Llevando a platos",
        }.get(self.estado, "Desconocido")


# ──────────────────────────────────────────────────────────────
#  BOT 4 – Repartidor
# ──────────────────────────────────────────────────────────────
class Bot4(BotBase):
    SPEED = 3.2

    def __init__(self):
        super().__init__()
        self.e = Entity(model='cube', color=color.violet,
                        position=Vec3(-1, _BOT_Y, 1),
                        scale=0.5, unlit=True, edge_color=color.black, edge_width=2)
        self.label = Text(text='B4', world_parent=self.e,
                          position=(0, 0.7, 0), scale=6,
                          billboard=True, color=color.white)
        self.estado = EstadoBot4.ESPERAR
        self.cargas = []

    def update(self, dt):
        global pedidos_completados, score
        if not juego_activo:
            return

        if self.estado == EstadoBot4.ESPERAR:
            nombres = [i.nombre for i in cola_platos]
            if (any('tomate_cortado' in n for n in nombres) and
                    any('lechuga_cortado' in n for n in nombres)):
                self.estado = EstadoBot4.IR_PLATO
                self._destino_final = None
                self.set_destino(acc_platos)
            else:
                self.estado = EstadoBot4.IR_IDLE
                self._destino_final = None
                self.set_destino(pos_idle_b4)
            return

        if self.estado == EstadoBot4.IR_IDLE:
            nombres = [i.nombre for i in cola_platos]
            if (any('tomate_cortado' in n for n in nombres) and
                    any('lechuga_cortado' in n for n in nombres)):
                self.estado = EstadoBot4.IR_PLATO
                self._destino_final = None
                self.set_destino(acc_platos)
            else:
                en_movimiento = self.mover(dt)
                if not en_movimiento:
                    apply_bot_separation(self, dt)
            return

        en_movimiento = self.mover(dt)

        # Actualizar posición de las cargas durante movimiento
        for idx, c in enumerate(self.cargas):
            c.position = self.e.position + Vec3((idx - 0.5) * 0.3, 0.6, 0)

        if en_movimiento:
            return

        if self.estado == EstadoBot4.IR_PLATO:
            tomado_t = tomado_l = False
            for item in cola_platos[:]:
                if not tomado_t and 'tomate_cortado' in item.nombre:
                    self.cargas.append(item)
                    cola_platos.remove(item)
                    tomado_t = True
                elif not tomado_l and 'lechuga_cortado' in item.nombre:
                    self.cargas.append(item)
                    cola_platos.remove(item)
                    tomado_l = True
                if tomado_t and tomado_l:
                    break
            if tomado_t and tomado_l:
                self.estado = EstadoBot4.IR_ENTREGA
                self._destino_final = None
                self.set_destino(acc_entrega)
            else:
                for c in self.cargas:
                    cola_platos.append(c)
                self.cargas = []
                self.estado = EstadoBot4.ESPERAR

        elif self.estado == EstadoBot4.IR_ENTREGA:
            for c in self.cargas:
                destroy(c)
            self.cargas = []
            pedidos_completados += 1
            score += 150
            actualizar_contador()
            actualizar_score()
            st_entrega.animate_scale(1.4, duration=0.15)
            st_entrega.animate_scale(1.0, duration=0.15, delay=0.15)
            self.estado = EstadoBot4.ESPERAR

    def get_status(self):
        return {
            EstadoBot4.ESPERAR: "Esperando",
            EstadoBot4.IR_IDLE: "Idle",
            EstadoBot4.IR_PLATO: "Recogiendo plato",
            EstadoBot4.IR_ENTREGA: "Entregando...",
        }.get(self.estado, "Desconocido")


# ──────────────────────────────────────────────────────────────
#  INSTANCIAR BOTS
# ──────────────────────────────────────────────────────────────
bot1 = Bot1()
bot2 = Bot2()
bot3 = Bot3()
bot4 = Bot4()

# ──────────────────────────────────────────────────────────────
#  UI
# ──────────────────────────────────────────────────────────────
window.color = color.dark_gray
window.fps_counter.enabled = False
window.exit_button.visible = False

Text(text="CONTROLES: WASD / Flechas  |  <red>R<default>: Nueva escena",
     position=(-0.75, 0.45), origin=(-0.5, 0.5),
     scale=1.1, background=True)

leyenda_texto = (
    "<red>Rojo:<default> Tomate\n"
    "<green>Verde:<default> Lechuga\n"
    "<yellow>Amarillo:<default> Corte\n"
    "<brown>Cafe:<default> Ensamblaje\n"
    "Blanco: Platos\n"
    "<azure>Azul:<default> Entrega\n"
    "Gris: Pared\n\n"
    "<orange>■<default> Bot1: Recolector\n"
    "<magenta>■<default> Bot2: Cortador\n"
    "<cyan>■<default> Bot3: Ensamblador\n"
    "<violet>■<default> Bot4: Repartidor"
)
Text(text=leyenda_texto, position=(0.55, 0.45),
     origin=(-0.5, 0.5), scale=0.85, background=True)

timer_text = Text(text="⏱ 1:30", position=(0, 0.46),
                  origin=(0, 0.5), scale=1.8,
                  background=True, color=color.yellow)

counter_text = Text(text="🍽 Pedidos: 0", position=(0, 0.36),
                    origin=(0, 0.5), scale=1.5,
                    background=True, color=color.lime)

score_text = Text(text="⭐ Puntos: 0", position=(0, 0.30),
                  origin=(0, 0.5), scale=1.5,
                  background=True, color=color.gold)

status_lines = []
for i in range(4):
    t = Text(text="", position=(-0.72, 0.35 - i*0.06),
             origin=(-0.5, 0.5), scale=0.9,
             background=True, color=color.white)
    status_lines.append(t)

fin_bg = Entity(model='quad', color=color.black66,
                scale=(0.9, 0.35), position=(0, 0),
                parent=camera.ui, enabled=False, z=-1)
fin_text = Text(text="", position=(0, 0), origin=(0, 0),
                scale=2.5, color=color.white, parent=camera.ui,
                enabled=False, z=-2)

def actualizar_contador():
    counter_text.text = f"🍽 Pedidos: {pedidos_completados}"

def actualizar_score():
    score_text.text = f"⭐ Puntos: {score}"

GEMINI_API_KEY = "AIzaSyCmxWwbV-35f9FwIiaYtS-baBZYxlV9ScQ"   # ← reemplaza esto con tu key
GEMINI_MODELS = [
    "gemini-2.5-flash",
    "gemini-2.5-pro",
    "gemini-2.0-flash-001",
    "gemini-1.5-flash-001",
    "gemini-1.5-pro-001",
]
GEMINI_BASE = "https://generativelanguage.googleapis.com/v1/models/{model}:generateContent?key={key}"

def _gemini_request(payload):
    """Intenta cada modelo en orden hasta que uno responda."""
    for model in GEMINI_MODELS:
        url = GEMINI_BASE.format(model=model, key=GEMINI_API_KEY)
        try:
            req = urllib.request.Request(
                url,
                data=json.dumps(payload).encode('utf-8'),
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=15) as resp:
                data = json.loads(resp.read().decode('utf-8'))
            print(f"[Gemini] Usando modelo: {model}")
            return data
        except urllib.request.HTTPError as e:
            body = ""
            try:
                body = e.read().decode('utf-8')
            except Exception:
                pass
            print(f"[Gemini] {model} → {e.code} {e.reason} — {body[:500]}")
        except Exception as e:
            print(f"[Gemini] {model} → Error: {e} — intentando siguiente...")
    return None


# ── La pared gris SIEMPRE arranca desde un borde y entra hacia el centro ──────
# Eje: 'z' = pared horizontal (sale del borde norte o sur, corre en X)
#       'x' = pared vertical   (sale del borde este u oeste, corre en Z)
# La pared deja siempre un pasillo de al menos WALL_GAP unidades en el extremo opuesto.
WALL_GAP = 1.2   # hueco mínimo entre punta de la pared y el borde opuesto

def _randomize_gray_wall():
    """
    Genera una pared gris que:
      - Está pegada a uno de los 4 bordes del mapa.
      - Se extiende hacia el interior dejando WALL_GAP de pasillo libre.
      - No obstruye ninguna estación (se le pasa la lista de spots para validar).
    Devuelve (center_x, z_min, z_max, eje) donde eje es 'NS' o 'EO'.
    """
    import random
    borde = half - 0.05   # coordenada del borde interior de la pared exterior

    # Elegir uno de los 4 bordes al azar
    lado = random.choice(['N', 'S', 'E', 'O'])
    largo_max = size - WALL_GAP - 0.3   # largo máximo sin tocar el borde opuesto
    largo = round(random.uniform(1.2, largo_max), 2)

    if lado == 'N':
        # Pared sale del borde norte (z = +half), corre en X, está en z fijo
        wall_z   = borde
        center_x = round(random.uniform(-half + 0.4, half - 0.4), 2)
        # Empieza en el borde norte y entra largo unidades hacia el sur
        z_max = wall_z
        z_min = round(wall_z - largo, 2)
        return center_x, z_min, z_max, 'NS'

    elif lado == 'S':
        wall_z   = -borde
        center_x = round(random.uniform(-half + 0.4, half - 0.4), 2)
        z_min = wall_z
        z_max = round(wall_z + largo, 2)
        return center_x, z_min, z_max, 'NS'

    elif lado == 'E':
        # Pared sale del borde este (x = +half), corre en Z, está en x fijo
        wall_x   = borde
        center_z = round(random.uniform(-half + 0.4, half - 0.4), 2)
        x_max = wall_x
        x_min = round(wall_x - largo, 2)
        return center_z, x_min, x_max, 'EO'   # reutilizamos los 3 valores

    else:  # O
        wall_x   = -borde
        center_z = round(random.uniform(-half + 0.4, half - 0.4), 2)
        x_min = wall_x
        x_max = round(wall_x + largo, 2)
        return center_z, x_min, x_max, 'EO'


def _apply_gray_wall(center, w_min, w_max, eje):
    """Actualiza la entidad gray_wall y las variables globales."""
    global gray_wall_center_x, GRAY_Z_MIN, GRAY_Z_MAX
    span = w_max - w_min
    mid  = (w_min + w_max) / 2
    if eje == 'NS':
        gray_wall_center_x = center
        GRAY_Z_MIN = w_min
        GRAY_Z_MAX = w_max
        gray_wall.position = (center, -half + 2.5, mid)
        gray_wall.scale    = (0.1, 5, span)
    else:  # EO: la pared corre en Z, el "center" es su posición en Z
        gray_wall_center_x = mid   # usamos el centro en X como referencia de bloqueo
        GRAY_Z_MIN = center - 0.05
        GRAY_Z_MAX = center + 0.05
        gray_wall.position = (mid, -half + 2.5, center)
        gray_wall.scale    = (span, 5, 0.1)


def call_gemini_for_scenario():
    """Llama a Gemini API para generar un escenario. Retorna dict o None."""
    prompt = (
        "Genera un escenario para un juego tipo Overcooked en una cuadrícula de 5x5 unidades "
        "(coordenadas válidas: x e z entre -2.3 y 2.3). "
        "Hay 4 bots que siguen esta cadena de producción:\n"
        "  Bot1 (Recolector): tomate → corte, lechuga → corte\n"
        "  Bot2 (Cortador):   corte → ensamblaje\n"
        "  Bot3 (Ensamblador):ensamblaje → platos\n"
        "  Bot4 (Repartidor): platos → entrega\n\n"
        "Necesito 6 estaciones pegadas a los bordes y una pared gris interna. "
        "Responde ÚNICAMENTE con un objeto JSON válido sin markdown:\n\n"
        "{\n"
        '  "stations": [\n'
        '    {"name": "tomate",     "x": number, "z": number},\n'
        '    {"name": "lechuga",    "x": number, "z": number},\n'
        '    {"name": "corte",      "x": number, "z": number},\n'
        '    {"name": "ensamblaje", "x": number, "z": number},\n'
        '    {"name": "platos",     "x": number, "z": number},\n'
        '    {"name": "entrega",    "x": number, "z": number}\n'
        "  ],\n"
        '  "gray_wall": {"side": "N"|"S"|"E"|"O", "offset": number, "length": number}\n'
        "}\n\n"
        "Restricciones OBLIGATORIAS:\n"
        "1. Estaciones en bordes: |x|>=1.8 O |z|>=1.8. Separadas >=1.0 entre sí.\n"
        "2. Pared gris pegada a un borde (side=N/S/E/O). "
        "   offset= posición lateral de la pared (-1.8 a 1.8). "
        "   length= largo de la pared (1.2 a 3.5). "
        "   La pared siempre arranca desde el borde elegido hacia adentro.\n"
        "3. Ninguna estación a menos de 0.9 unidades de la pared.\n"
        "4. El flujo tomate/lechuga→corte→ensamblaje→platos→entrega debe tener rutas libres.\n"
        "Genera valores diversos cada vez."
    )

    payload = {
        "contents": [{"parts": [{"text": prompt}]}],
        "generationConfig": {"temperature": 0.9, "maxOutputTokens": 2048}
    }

    data = _gemini_request(payload)
    if data is None:
        return None

    try:
        text = data["candidates"][0]["content"]["parts"][0]["text"]
        text = text.strip()
        print("[Gemini RAW]", text[:600])
        if text.startswith("```json"):
            text = text[7:]
        if text.startswith("```"):
            text = text[3:]
        if text.endswith("```"):
            text = text[:-3]
        text = text.strip()
        scenario = json.loads(text)
        return scenario
    except Exception as e:
        print("[Gemini] Parse error:", e)
        return None


def _manual_randomize():
    """Randomización manual de las estaciones (la pared ya fue aplicada antes)."""
    import random

    margin = 0.5
    border = half - margin
    step = 0.8
    coords = [round(i * step, 2) for i in range(-2, 3)]
    candidates = []
    for c in coords:
        candidates.append((c, border))
        candidates.append((c, -border))
        candidates.append((border, c))
        candidates.append((-border, c))
    seen = set()
    unique = []
    for cand in candidates:
        if cand not in seen:
            seen.add(cand)
            unique.append(cand)
    candidates = unique
    random.shuffle(candidates)

    st_half = STATION_SCALE * 0.5
    safe = st_half + 0.05 + 0.4
    filtered = []
    for x, z in candidates:
        too_close = False
        # Evitar estaciones sobre la pared gris
        if abs(x - gray_wall_center_x) < safe:
            if (z > GRAY_Z_MIN - safe) and (z < GRAY_Z_MAX + safe):
                too_close = True
        if not too_close:
            filtered.append((x, z))

    spots = []
    for cand in filtered:
        ok = True
        for s in spots:
            if ((cand[0] - s[0]) ** 2 + (cand[1] - s[1]) ** 2) ** 0.5 < 1.0:
                ok = False
                break
        if ok:
            spots.append(cand)
        if len(spots) >= 6:
            break

    if len(spots) < 6:
        spots = [
            (1.6, border), (-0.8, border), (0.8, -border),
            (-1.6, -border), (border, 0.8), (-border, -0.8),
        ]
    return spots


def randomize_scenario():
    global pos_tomate, pos_lechuga, pos_corte, pos_ensamblaje, pos_platos, pos_entrega
    global acc_tomate, acc_lechuga, acc_corte, acc_ensamblaje, acc_platos, acc_entrega
    global NAV_GRID, DANGER_GRID, static_obstacles
    global tiempo_restante, juego_activo, pedidos_completados, score
    global cola_corte, cola_ensamblaje, cola_platos, plato_actual, all_ingredientes
    global gray_wall_center_x, GRAY_Z_MIN, GRAY_Z_MAX

    scenario = call_gemini_for_scenario()
    spots = None

    if scenario and "stations" in scenario and "gray_wall" in scenario:
        try:
            gw   = scenario["gray_wall"]
            lado = str(gw["side"]).upper().strip()
            offset = float(gw["offset"])
            length = float(gw["length"])
            if lado not in ('N', 'S', 'E', 'O'):
                raise ValueError(f"side inválido: {lado}")
            if length < 0.8:
                raise ValueError("length demasiado corta")

            borde = half - 0.05
            if lado == 'N':
                w_min = round(borde - length, 2);  w_max = borde
                _apply_gray_wall(offset, w_min, w_max, 'NS')
            elif lado == 'S':
                w_min = -borde;  w_max = round(-borde + length, 2)
                _apply_gray_wall(offset, w_min, w_max, 'NS')
            elif lado == 'E':
                w_min = round(borde - length, 2);  w_max = borde
                _apply_gray_wall(offset, w_min, w_max, 'EO')
            else:  # O
                w_min = -borde;  w_max = round(-borde + length, 2)
                _apply_gray_wall(offset, w_min, w_max, 'EO')

            name_map = {"tomate": None, "lechuga": None, "corte": None,
                        "ensamblaje": None, "platos": None, "entrega": None}
            for st in scenario["stations"]:
                n = st["name"].lower().strip()
                if n in name_map:
                    name_map[n] = (float(st["x"]), float(st["z"]))

            if None in name_map.values():
                raise ValueError("Faltan estaciones en la respuesta de Gemini")

            spots = [
                name_map["tomate"], name_map["lechuga"], name_map["corte"],
                name_map["ensamblaje"], name_map["platos"], name_map["entrega"]
            ]
            print("[Gemini] Escenario generado por IA")
        except Exception as e:
            print("[Gemini] Parse error:", e, "— usando random manual")
            spots = None

    if spots is None:
        print("[Gemini] Sin respuesta válida — usando random manual")
        # Pared pegada al borde (manual)
        import random
        centro, w_min, w_max, eje = _randomize_gray_wall()
        _apply_gray_wall(centro, w_min, w_max, eje)
        spots = _manual_randomize()

    def _set_station(st_entity, new_pos):
        st_entity.position = new_pos
        return new_pos

    pos_tomate     = _set_station(st_tomate,     Vec3(spots[0][0], STATION_Y, spots[0][1]))
    pos_lechuga    = _set_station(st_lechuga,    Vec3(spots[1][0], STATION_Y, spots[1][1]))
    pos_corte      = _set_station(st_corte,      Vec3(spots[2][0], STATION_Y, spots[2][1]))
    pos_ensamblaje = _set_station(st_ensamblaje, Vec3(spots[3][0], STATION_Y, spots[3][1]))
    pos_platos     = _set_station(st_platos,     Vec3(spots[4][0], STATION_Y, spots[4][1]))
    pos_entrega    = _set_station(st_entrega,    Vec3(spots[5][0], STATION_Y, spots[5][1]))

    station_positions = [pos_tomate, pos_lechuga, pos_corte, pos_ensamblaje, pos_platos, pos_entrega]
    NAV_GRID, DANGER_GRID = _build_grids(station_positions)

    static_obstacles.clear()
    for sp in station_positions:
        register_obstacle(sp)

    def _compute_access(station_pos):
        to_center = Vec3(0, _BOT_Y, 0) - station_pos
        to_center.y = 0
        if to_center.length() < 0.1:
            to_center = Vec3(0, 0, 1)
        to_center = to_center.normalized()
        acc = station_pos + to_center * 0.7
        return Vec3(acc.x, _BOT_Y, acc.z)

    acc_tomate     = _compute_access(pos_tomate)
    acc_lechuga    = _compute_access(pos_lechuga)
    acc_corte      = _compute_access(pos_corte)
    acc_ensamblaje = _compute_access(pos_ensamblaje)
    acc_platos     = _compute_access(pos_platos)
    acc_entrega    = _compute_access(pos_entrega)

    tiempo_restante = TIEMPO_LIMITE
    juego_activo = True
    pedidos_completados = 0
    score = 0
    timer_text.color = color.yellow
    fin_bg.enabled = False
    fin_text.enabled = False

    cola_corte.clear()
    cola_ensamblaje.clear()
    cola_platos.clear()
    plato_actual.clear()

    for bot in all_bots:
        bot.waypoints.clear()
        bot._destino_final = None
        bot._last_pos = None
        bot._stuck_t = 0.0
        if hasattr(bot, 'carga') and bot.carga:
            bot.carga = None
        if hasattr(bot, 'cargas'):
            bot.cargas.clear()
        if hasattr(bot, 'timer'):
            bot.timer = 0.0

    for ing in all_ingredientes:
        if ing:
            destroy(ing)
    all_ingredientes.clear()

    bot1.e.position = Vec3(0, _BOT_Y, 2)
    bot1.e.rotation_y = 0
    bot1.estado = EstadoBot1.IR_TOMATE
    bot1.set_destino(acc_tomate)

    bot2.e.position = Vec3(0, _BOT_Y, 1.5)
    bot2.e.rotation_y = 0
    bot2.estado = EstadoBot2.ESPERAR

    bot3.e.position = Vec3(1, _BOT_Y, 0)
    bot3.e.rotation_y = 0
    bot3.estado = EstadoBot3.ESPERAR

    bot4.e.position = Vec3(-1, _BOT_Y, 1)
    bot4.e.rotation_y = 0
    bot4.estado = EstadoBot4.ESPERAR

    actualizar_contador()
    actualizar_score()

def mostrar_fin():
    fin_bg.enabled   = True
    fin_text.enabled = True
    fin_text.text    = (f"⏰ ¡TIEMPO!\n"
                        f"Pedidos completados: {pedidos_completados}")
    fin_text.color   = color.yellow

# ──────────────────────────────────────────────────────────────
#  CÁMARA ORBITAL
# ──────────────────────────────────────────────────────────────
pivot = Entity()
camera.parent   = pivot
camera.position = (0, 0, -18)
pivot.rotation_x, pivot.rotation_y = 35, 45

# ──────────────────────────────────────────────────────────────
#  UPDATE PRINCIPAL
# ──────────────────────────────────────────────────────────────
_r_pressed = False

def update():
    global tiempo_restante, juego_activo, _r_pressed

    dt = time.dt

    if juego_activo:
        tiempo_restante -= dt
        if tiempo_restante <= 0:
            tiempo_restante = 0
            juego_activo    = False
            mostrar_fin()

        mins = int(tiempo_restante) // 60
        segs = int(tiempo_restante) % 60
        timer_text.text = f"⏱ {mins}:{segs:02d}"

        if tiempo_restante < 20:
            timer_text.color = color.red
        elif tiempo_restante < 40:
            timer_text.color = color.orange
        else:
            timer_text.color = color.yellow

    bot1.update(dt)
    bot2.update(dt)
    bot3.update(dt)
    bot4.update(dt)

    if held_keys['r']:
        if not _r_pressed:
            _r_pressed = True
            randomize_scenario()
    else:
        _r_pressed = False

    status_lines[0].text = f"<orange>B1<default>: {bot1.get_status()}"
    status_lines[1].text = f"<magenta>B2<default>: {bot2.get_status()}"
    status_lines[2].text = f"<cyan>B3<default>: {bot3.get_status()}"
    status_lines[3].text = f"<violet>B4<default>: {bot4.get_status()}"

    rot_speed = 100 * dt
    pivot.rotation_y += (held_keys['d'] - held_keys['a'] +
                         held_keys['right arrow'] - held_keys['left arrow']) * rot_speed
    pivot.rotation_x += (held_keys['w'] - held_keys['s'] +
                         held_keys['up arrow'] - held_keys['down arrow']) * rot_speed
    pivot.rotation_x  = clamp(pivot.rotation_x, 10, 85)

    cam_pos = camera.world_position
    for w in walls:
        if w.eje == 'x':
            w.enabled = not ((w.dir ==  1 and cam_pos.x > w.position.x) or
                             (w.dir == -1 and cam_pos.x < w.position.x))
        elif w.eje == 'z':
            w.enabled = not ((w.dir ==  1 and cam_pos.z > w.position.z) or
                             (w.dir == -1 and cam_pos.z < w.position.z))

app.run()
