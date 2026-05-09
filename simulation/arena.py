"""
simulation/arena.py — Carga map_config.json y construye el grid 3D 5x5.
Cada celda = cubo de 5x5x5m. Paredes entre celdas según configuración.
"""

import json
from ursina import Entity, Vec3, color
from .physics import PhysicsWorld

STATION_MARKER_COLORS = {
    "almacen":    color.rgb(50, 180, 50),
    "tablero":    color.rgb(60, 60, 120),
    "corte":      color.rgb(160, 160, 170),
    "ensamblaje": color.rgb(220, 170, 40),
    "entrega":    color.rgb(40, 140, 40),
    "obstacle":   color.rgb(80, 80, 80),
}


class Arena:

    def __init__(self, physics: PhysicsWorld, config_path: str = "map_config.json"):
        self.physics = physics
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = json.load(f)

        self.rows = self.config["grid_rows"]
        self.cols = self.config["grid_cols"]
        self.cell_size = self.config["cell_size"]  # [w, h, d]
        self.wall_height = self.config["wall_height"]
        self.cw, self.ch, self.cd = self.cell_size

        self.cell_data = {}
        for c in self.config["cells"]:
            self.cell_data[(c["row"], c["col"])] = c

        self._build_floor()
        self._build_walls()
        self._build_station_markers()

        print(f"[Arena] Grid {self.rows}x{self.cols} | cells={len(self.cell_data)}")

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

    def _build_floor(self):
        floor_color = color.rgb(25, 30, 35)
        for row in range(self.rows):
            for col in range(self.cols):
                cx, _, cz = self.cell_center_3d(row, col)
                cell = self.cell_data.get((row, col))
                clr = floor_color
                if cell and "color" in cell:
                    clr = color.rgb(
                        int(cell["color"][0] * 255),
                        int(cell["color"][1] * 255),
                        int(cell["color"][2] * 255))
                Entity(
                    model="cube", color=clr,
                    scale=(self.cw * 0.98, 0.05, self.cd * 0.98),
                    position=Vec3(cx, -0.03, cz),
                )

    def _build_walls(self):
        wt = 0.2
        wh = self.wall_height
        hw = wh / 2
        for row in range(self.rows):
            for col in range(self.cols):
                cx, _, cz = self.cell_center_3d(row, col)
                cell = self.cell_data.get((row, col))

                if cell and cell.get("type") == "obstacle":
                    self.physics.create_box(
                        half_extents=(self.cw/2, wh/2, wt/2), mass=0,
                        position=(cx, hw, cz - self.cd/2))
                    self.physics.create_box(
                        half_extents=(wt/2, wh/2, self.cd/2), mass=0,
                        position=(cx - self.cw/2, hw, cz))
                    Entity(model="cube", color=color.rgb(100, 40, 40),
                           scale=(self.cw, wh, wt),
                           position=Vec3(cx, hw, cz - self.cd/2))
                    Entity(model="cube", color=color.rgb(100, 40, 40),
                           scale=(wt, wh, self.cd),
                           position=Vec3(cx - self.cw/2, hw, cz))
                    continue

                if col < self.cols - 1:
                    neighbor = self.cell_data.get((row, col + 1))
                    if self._needs_wall(cell, neighbor):
                        wall_x = cx + self.cw / 2
                        self.physics.create_box(
                            half_extents=(wt/2, wh/2, self.cd/2), mass=0,
                            position=(wall_x, hw, cz))
                        Entity(model="cube", color=color.rgb(60, 60, 70),
                               scale=(wt, wh, self.cd),
                               position=Vec3(wall_x, hw, cz))

                if row < self.rows - 1:
                    neighbor = self.cell_data.get((row + 1, col))
                    if self._needs_wall(cell, neighbor):
                        wall_z = cz + self.cd / 2
                        self.physics.create_box(
                            half_extents=(self.cw/2, wh/2, wt/2), mass=0,
                            position=(cx, hw, wall_z))
                        Entity(model="cube", color=color.rgb(60, 60, 70),
                               scale=(self.cw, wh, wt),
                               position=Vec3(cx, hw, wall_z))

    def _needs_wall(self, cell_a, cell_b):
        type_a = cell_a.get("type") if cell_a else "pasillo"
        type_b = cell_b.get("type") if cell_b else "pasillo"
        if type_a == "obstacle" or type_b == "obstacle":
            return True
        if type_a in ("corte", "ensamblaje", "entrega") and type_b == "pasillo":
            return True
        if type_b in ("corte", "ensamblaje", "entrega") and type_a == "pasillo":
            return True
        return False

    def _build_station_markers(self):
        for cell in self.config["cells"]:
            row, col = cell["row"], cell["col"]
            tp = cell["type"]
            if tp == "obstacle":
                continue
            cx, _, cz = self.cell_center_3d(row, col)
            marker_color = STATION_MARKER_COLORS.get(tp, color.white)
            Entity(model="cylinder", color=marker_color,
                   scale=(0.2, 2.5, 0.2),
                   position=Vec3(cx + self.cw*0.35, 1.25, cz))
            label_text = cell.get("label", tp)
            Entity(model="quad", color=color.white,
                   scale=(1.5, 0.5, 1),
                   position=Vec3(cx, 3.0, cz), billboard=True)
