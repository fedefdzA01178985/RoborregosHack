"""
simulation/arena.py — Arena 3D con texturas PIL, paredes quads, estilo unlit.
Grid 5x5 de 5x5x5m. Paredes con textura de rejilla. Suelo con grid.
Los objetos de estacion usan unlit + edge_color (estilo dibujo).
"""

import json
from ursina import Entity, Vec3, color, Texture
from PIL import Image, ImageDraw
from .physics import PhysicsWorld


def generate_grid_tex(bg_rgba, line_rgba, grid_size=5, res=1024):
    img = Image.new("RGBA", (res, res), bg_rgba)
    draw = ImageDraw.Draw(img)
    lw = 6
    for i in range(grid_size + 1):
        pos = int((i / grid_size) * (res - lw / 2))
        draw.line((0, pos, res, pos), fill=line_rgba, width=lw)
        draw.line((pos, 0, pos, res), fill=line_rgba, width=lw)
    return Texture(img)


CELL_FLOOR_COLORS = {
    "almacen":    color.rgb(40, 140, 40),
    "tablero":    color.rgb(50, 50, 100),
    "corte":      color.rgb(130, 130, 140),
    "ensamblaje": color.rgb(200, 150, 30),
    "entrega":    color.rgb(30, 120, 30),
    "obstacle":   color.rgb(180, 40, 40),
    "pasillo":    color.rgb(45, 50, 60),
}

STATION_COLORS = {
    "almacen":    color.rgb(60, 180, 60),
    "tablero":    color.rgb(80, 80, 180),
    "corte":      color.rgb(180, 180, 190),
    "ensamblaje": color.rgb(240, 180, 40),
    "entrega":    color.rgb(40, 160, 40),
}


class Arena:

    def __init__(self, physics: PhysicsWorld, config_path: str = "map_config.json"):
        self.physics = physics
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = json.load(f)

        self.rows = self.config["grid_rows"]
        self.cols = self.config["grid_cols"]
        self.cell_size = self.config["cell_size"]
        self.wall_height = self.config["wall_height"]
        self.cw, self.ch, self.cd = self.cell_size

        self.arena_w = self.cols * self.cw
        self.arena_d = self.rows * self.cd
        self.hw = self.arena_w / 2
        self.hd = self.arena_d / 2
        self.wh = self.wall_height

        self.cell_data = {}
        for c in self.config["cells"]:
            self.cell_data[(c["row"], c["col"])] = c

        self._build_textures()
        self._build_floor()
        self._build_wall_quads()
        self._build_cell_floors()
        self._build_stations()
        self._build_perimeter_physics()

        print(f"[Arena] Grid {self.rows}x{self.cols} | {self.arena_w}x{self.arena_d}m | cells={len(self.cell_data)}")

    def cell_center(self, row, col):
        cx = (col - (self.cols - 1) / 2) * self.cw
        cz = (row - (self.rows - 1) / 2) * self.cd
        return Vec3(cx, 0, cz)

    def cell_center_3d(self, row, col):
        cx = (col - (self.cols - 1) / 2) * self.cw
        cz = (row - (self.rows - 1) / 2) * self.cd
        return (cx, 0.5, cz)

    def is_passable(self, row, col):
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return False
        cell = self.cell_data.get((row, col))
        if cell and cell.get("type") == "obstacle":
            return False
        return True

    def get_cell_at(self, world_x, world_z):
        col = round(world_x / self.cw + (self.cols - 1) / 2)
        row = round(world_z / self.cd + (self.rows - 1) / 2)
        row = max(0, min(self.rows - 1, row))
        col = max(0, min(self.cols - 1, col))
        return (row, col)

    def get_cell_type(self, row, col):
        cell = self.cell_data.get((row, col))
        return cell["type"] if cell else "pasillo"

    def get_cell_label(self, row, col):
        cell = self.cell_data.get((row, col))
        return cell.get("label", "") if cell else ""

    def get_stations_by_type(self, station_type):
        return [(c["row"], c["col"]) for c in self.config["cells"]
                if c.get("type") == station_type]

    # ── Texturas ───────────────────────────────────────────────────────

    def _build_textures(self):
        bg_floor = (35, 38, 48, 255)
        ln_grid = (220, 220, 235, 200)
        self.floor_tex = generate_grid_tex(bg_floor, ln_grid, grid_size=self.rows)

        bg_wall = (130, 155, 185, 255)
        ln_wall = (210, 220, 235, 200)
        self.wall_tex = generate_grid_tex(bg_wall, ln_wall, grid_size=self.rows)

    # ── Suelo ──────────────────────────────────────────────────────────

    def _build_floor(self):
        Entity(
            model="quad", scale=(self.arena_w, self.arena_d),
            position=(0, 0, 0), rotation=(-90, 0, 0),
            texture=self.floor_tex, double_sided=True,
        )

    # ── Paredes visuales (quads) ────────────────────────────────────────

    def _build_wall_quads(self):
        faces = [
            {"pos": (0, self.wh / 2, self.hd),  "rot": (0, 180, 0),  "eje": "z", "dir": 1},
            {"pos": (0, self.wh / 2, -self.hd), "rot": (0, 0, 0),    "eje": "z", "dir": -1},
            {"pos": (self.hw, self.wh / 2, 0),  "rot": (0, 90, 0),   "eje": "x", "dir": 1},
            {"pos": (-self.hw, self.wh / 2, 0), "rot": (0, -90, 0),  "eje": "x", "dir": -1},
        ]
        self.wall_quads = []
        for f in faces:
            w = Entity(
                model="quad", scale=(self.arena_w, self.wh),
                position=f["pos"], rotation=f["rot"],
                texture=self.wall_tex, double_sided=True,
            )
            w.eje, w.dir = f["eje"], f["dir"]
            self.wall_quads.append(w)

    # ── Pisos de celdas ────────────────────────────────────────────────

    def _build_cell_floors(self):
        for row in range(self.rows):
            for col in range(self.cols):
                cx, _, cz = self.cell_center_3d(row, col)
                cell = self.cell_data.get((row, col))
                tp = cell.get("type", "pasillo") if cell else "pasillo"
                clr = CELL_FLOOR_COLORS.get(tp, CELL_FLOOR_COLORS["pasillo"])
                Entity(
                    model="cube", color=clr,
                    scale=(self.cw * 0.94, 0.04, self.cd * 0.94),
                    position=Vec3(cx, 0.01, cz),
                    unlit=True, edge_color=color.black, edge_width=1,
                )

    # ── Estaciones (cubos unlit con borde) ──────────────────────────────

    def _build_stations(self):
        for cell in self.config["cells"]:
            row, col = cell["row"], cell["col"]
            tp = cell["type"]
            if tp == "obstacle":
                self._build_obstacle_cube(row, col)
                continue
            cx, _, cz = self.cell_center_3d(row, col)
            st_color = STATION_COLORS.get(tp, color.white)
            Entity(
                model="cube", color=st_color,
                position=Vec3(cx, 0.6, cz), scale=1.2,
                unlit=True, edge_color=color.black, edge_width=3,
            )

    def _build_obstacle_cube(self, row, col):
        cx, _, cz = self.cell_center_3d(row, col)
        Entity(
            model="cube", color=color.rgb(190, 50, 50),
            position=Vec3(cx, self.wall_height / 2, cz),
            scale=(self.cw * 0.85, self.wall_height, self.cd * 0.85),
            unlit=True, edge_color=color.black, edge_width=2,
        )

    # ── Fi­sica (PyBullet) ──────────────────────────────────────────────

    def _build_perimeter_physics(self):
        wt = 0.3
        wh = self.wall_height
        hw = wh / 2
        ahw = self.hw
        ahd = self.hd
        segments = [
            (ahw + wt/2, hw, 0, wt/2, wh/2, ahd + wt),
            (-(ahw + wt/2), hw, 0, wt/2, wh/2, ahd + wt),
            (0, hw, ahd + wt/2, ahw + wt, wh/2, wt/2),
            (0, hw, -(ahd + wt/2), ahw + wt, wh/2, wt/2),
        ]
        for wx, wy, wz, hx, hy, hz in segments:
            self.physics.create_box(
                half_extents=(hx, hy, hz), mass=0,
                position=(wx, wy, wz), friction=0.3)

        for (row, col), cell in self.cell_data.items():
            if cell.get("type") != "obstacle":
                continue
            cx, _, cz = self.cell_center_3d(row, col)
            for bx, bz in [(0, -self.cd/2), (0, self.cd/2),
                           (-self.cw/2, 0), (self.cw/2, 0)]:
                hx = self.cw/2 if bx == 0 else wt/2
                hz = self.cd/2 if bz == 0 else wt/2
                self.physics.create_box(
                    half_extents=(hx, wh/2, hz), mass=0,
                    position=(cx + bx, wh/2, cz + bz), friction=0.3)
