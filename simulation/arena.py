"""
simulation/arena.py — Carga map_config.json y construye el grid 3D 5x5.
Cada celda = cubo de 5x5x5m. Paredes visibles. Lineas de grid. Perimetro.
"""

import json
from ursina import Entity, Vec3, color, Text
from .physics import PhysicsWorld

CELL_FLOOR_COLORS = {
    "almacen":    color.rgb(40, 140, 40),
    "tablero":    color.rgb(50, 50, 100),
    "corte":      color.rgb(130, 130, 140),
    "ensamblaje": color.rgb(200, 150, 30),
    "entrega":    color.rgb(30, 120, 30),
    "obstacle":   color.rgb(180, 40, 40),
    "pasillo":    color.rgb(45, 50, 60),
}

STATION_TEXT_COLORS = {
    "almacen":    color.rgb(150, 255, 150),
    "tablero":    color.rgb(180, 180, 255),
    "corte":      color.rgb(220, 220, 220),
    "ensamblaje": color.rgb(255, 230, 120),
    "entrega":    color.rgb(120, 255, 120),
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

        self.cell_data = {}
        for c in self.config["cells"]:
            self.cell_data[(c["row"], c["col"])] = c

        self._build_ground_plane()
        self._build_cell_floors()
        self._build_grid_lines()
        self._build_perimeter_walls()
        self._build_internal_walls()
        self._build_station_labels()

        print(f"[Arena] Grid {self.rows}x{self.cols} | cells={len(self.cell_data)}")

    def cell_center(self, row, col):
        cx = (col - (self.cols - 1) / 2) * self.cw
        cz = (row - (self.rows - 1) / 2) * self.cd
        return Vec3(cx, 0, cz)

    def cell_center_3d(self, row, col):
        cx = (col - (self.cols - 1) / 2) * self.cw
        cz = (row - (self.rows - 1) / 2) * self.cd
        return (cx, 0.5, cz)

    def arena_half_w(self):
        return self.cols * self.cw / 2

    def arena_half_d(self):
        return self.rows * self.cd / 2

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

    def _build_ground_plane(self):
        hw = self.arena_half_w() + 2
        hd = self.arena_half_d() + 2
        Entity(
            model="cube", color=color.rgb(12, 14, 20),
            scale=(hw * 2, 0.1, hd * 2),
            position=Vec3(0, -0.1, 0),
        )

    def _build_cell_floors(self):
        for row in range(self.rows):
            for col in range(self.cols):
                cx, _, cz = self.cell_center_3d(row, col)
                cell = self.cell_data.get((row, col))
                tp = cell.get("type", "pasillo") if cell else "pasillo"
                clr = CELL_FLOOR_COLORS.get(tp, CELL_FLOOR_COLORS["pasillo"])
                Entity(
                    model="cube", color=clr,
                    scale=(self.cw * 0.96, 0.06, self.cd * 0.96),
                    position=Vec3(cx, 0.02, cz),
                )

    def _build_grid_lines(self):
        line_color = color.rgb(200, 200, 220)
        hw = self.arena_half_w()
        hd = self.arena_half_d()
        thin = 0.04
        for row in range(self.rows + 1):
            z = hd - row * self.cd
            Entity(model="cube", color=line_color,
                   scale=(hw * 2, 0.03, thin),
                   position=Vec3(0, 0.05, z))
        for col in range(self.cols + 1):
            x = -hw + col * self.cw
            Entity(model="cube", color=line_color,
                   scale=(thin, 0.03, hd * 2),
                   position=Vec3(x, 0.05, 0))

    def _build_perimeter_walls(self):
        wt = 0.3
        wh = self.wall_height
        hw = wh / 2
        ahw = self.arena_half_w()
        ahd = self.arena_half_d()
        wall_color = color.rgb(70, 75, 90)
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
            Entity(model="cube", color=wall_color,
                   scale=(hx * 2, hy * 2, hz * 2),
                   position=Vec3(wx, wy, wz))

    def _build_internal_walls(self):
        wt = 0.2
        wh = self.wall_height
        hw = wh / 2
        wall_color = color.rgb(100, 105, 120)

        for row in range(self.rows):
            for col in range(self.cols):
                cx, _, cz = self.cell_center_3d(row, col)
                cell = self.cell_data.get((row, col))

                if cell and cell.get("type") == "obstacle":
                    self.physics.create_box(
                        half_extents=(self.cw/2, wh/2, wt/2), mass=0,
                        position=(cx, hw, cz - self.cd/2))
                    self.physics.create_box(
                        half_extents=(self.cw/2, wh/2, wt/2), mass=0,
                        position=(cx, hw, cz + self.cd/2))
                    self.physics.create_box(
                        half_extents=(wt/2, wh/2, self.cd/2), mass=0,
                        position=(cx - self.cw/2, hw, cz))
                    self.physics.create_box(
                        half_extents=(wt/2, wh/2, self.cd/2), mass=0,
                        position=(cx + self.cw/2, hw, cz))
                    Entity(model="cube", color=color.rgb(180, 40, 40),
                           scale=(self.cw, wh, wt),
                           position=Vec3(cx, hw, cz - self.cd/2))
                    Entity(model="cube", color=color.rgb(180, 40, 40),
                           scale=(self.cw, wh, wt),
                           position=Vec3(cx, hw, cz + self.cd/2))
                    Entity(model="cube", color=color.rgb(180, 40, 40),
                           scale=(wt, wh, self.cd),
                           position=Vec3(cx - self.cw/2, hw, cz))
                    Entity(model="cube", color=color.rgb(180, 40, 40),
                           scale=(wt, wh, self.cd),
                           position=Vec3(cx + self.cw/2, hw, cz))
                    continue

                if col < self.cols - 1 and self._needs_wall(cell, row, col + 1):
                    wx = cx + self.cw / 2
                    self.physics.create_box(
                        half_extents=(wt/2, wh/2, self.cd/2), mass=0,
                        position=(wx, hw, cz))
                    Entity(model="cube", color=wall_color,
                           scale=(wt, wh, self.cd),
                           position=Vec3(wx, hw, cz))

                if row < self.rows - 1 and self._needs_wall(cell, row + 1, col):
                    wz = cz + self.cd / 2
                    self.physics.create_box(
                        half_extents=(self.cw/2, wh/2, wt/2), mass=0,
                        position=(cx, hw, wz))
                    Entity(model="cube", color=wall_color,
                           scale=(self.cw, wh, wt),
                           position=Vec3(cx, hw, wz))

    def _needs_wall(self, cell_a, row_b, col_b):
        type_a = cell_a.get("type") if cell_a else "pasillo"
        cell_b = self.cell_data.get((row_b, col_b))
        type_b = cell_b.get("type") if cell_b else "pasillo"
        if type_a == "obstacle" or type_b == "obstacle":
            return True
        return False

    def _build_station_labels(self):
        for cell in self.config["cells"]:
            row, col = cell["row"], cell["col"]
            tp = cell["type"]
            if tp == "obstacle":
                continue
            cx, _, cz = self.cell_center_3d(row, col)
            label = cell.get("label", tp).upper()
            txt_color = STATION_TEXT_COLORS.get(tp, color.white)
            Entity(model="quad", color=color.rgba(0, 0, 0, 200),
                   scale=(2.5, 0.7, 1),
                   position=Vec3(cx, 3.6, cz), billboard=True)
            Text(text=label, position=Vec3(cx, 3.6, cz),
                 origin=(0, 0), scale=1.2, color=txt_color, billboard=True)
