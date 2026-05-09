"""
RoboKitchen — MLH Hackathon 2026 | RoBorregos | Tec de Monterrey

Cocina robotica 3D: 4 robots en cadena preparan ensaladas.
Grid 5x5 de celdas de 5x5x5m. Gemini supervisa el pipeline.

Stack: Ursina (render) + PyBullet (fisica) + Gemini API (IA)

Controles debug:
  WASD = mover robot recolector
  R    = reset
  G    = toggle Gemini ON/OFF
  ESC  = salir
"""

import threading
import time
import sys

try:
    from ursina import (Ursina, Entity, Vec3, color, time as utime,
                        held_keys, Text, camera, window, DirectionalLight,
                        AmbientLight)
except ImportError:
    print("[ERROR] Ursina no instalado. pip install ursina")
    sys.exit(1)

try:
    import pybullet as p
except ImportError:
    print("[ERROR] PyBullet no instalado. pip install pybullet")
    sys.exit(1)

from simulation import PhysicsWorld, Robot, Ingredient, Arena, PathFinder
from gemini import GeminiAgent
from orders import OrderManager
from ui import HUD, Scoreboard

CONFIG = {
    "window_title": "RoboKitchen — MLH Hackathon 2026 | RoBorregos",
    "window_size": (1280, 720),
    "fullscreen": False,
    "gemini_interval": 3.0,
    "physics_hz": 240,
    "score_limit": 10,
}


class RoboKitchen:

    def __init__(self):
        self.score = 0
        self.game_active = True
        self.gemini_enabled = True
        self._physics_lock = threading.Lock()
        self._last_gemini_call = 0.0
        self._pending_commands = []
        self._physics_step_time = 1.0 / CONFIG["physics_hz"]

        self._init_ursina()
        self._init_physics()
        self._init_arena()
        self._init_ingredients()
        self._init_robots()
        self._init_orders()
        self._init_gemini()
        self._init_ui()

        self._assembled_ingredients = []
        self.pathfinder = PathFinder(5, 5)
        self._build_passability_grid()

        print("\n[RoboKitchen] Iniciado")
        print(f"  Gemini: {'ON' if self.gemini_enabled else 'OFF'}")
        print(f"  Pedidos pendientes: {self.order_manager.pending}")

    def _init_ursina(self):
        self.app = Ursina(
            title=CONFIG["window_title"],
            size=CONFIG["window_size"],
            fullscreen=CONFIG["fullscreen"],
            development_mode=True,
            borderless=False,
        )
        window.fps_counter.enabled = True
        window.exit_button.visible = False
        window.color = color.rgb(15, 18, 25)

        camera.position = Vec3(-1, 22, -12)
        camera.rotation_x = 60
        camera.fov = 55

        AmbientLight(color=color.rgba(200, 200, 220, 255))
        main_light = DirectionalLight()
        main_light.look_at(Vec3(0.5, -1, -0.5))

    def _init_physics(self):
        self.physics = PhysicsWorld()

    def _init_arena(self):
        self.arena = Arena(self.physics, "map_config.json")

    def _init_ingredients(self):
        self.ingredients = []
        for cell in self.arena.config["cells"]:
            if cell.get("type") == "almacen" and "spawn" in cell:
                wx, wy, wz = self.arena.cell_center_3d(cell["row"], cell["col"])
                ing = Ingredient(self.physics, cell["spawn"],
                                 (cell["row"], cell["col"]), (wx, wy, wz))
                self.ingredients.append(ing)

    def _init_robots(self):
        self.robots = []
        for rc in self.arena.config["robots"]:
            row, col = rc["start_cell"]
            wx, wy, wz = self.arena.cell_center_3d(row, col)
            robot = Robot(self.physics, rc["id"], rc["role"],
                          (row, col), (wx, wy, wz))
            self.robots.append(robot)

    def _init_orders(self):
        self.order_manager = OrderManager(points_per_order=150)
        self.order_manager.spawn()

    def _init_gemini(self):
        try:
            self.agent = GeminiAgent()
        except Exception as e:
            print(f"[Gemini] No disponible: {e}")
            self.gemini_enabled = False
            self.agent = None

    def _init_ui(self):
        self.hud = HUD()
        self.scoreboard = Scoreboard()

    def _build_passability_grid(self):
        self.passable = [[True] * 5 for _ in range(5)]
        for (row, col), cell in self.arena.cell_data.items():
            if cell.get("type") == "obstacle":
                self.passable[row][col] = False

    def update(self):
        if not self.game_active:
            return
        dt = utime.dt
        if dt <= 0 or dt > 0.1:
            return

        self._handle_debug_input()
        self._step_physics(dt)
        self._call_gemini_if_ready()
        self._apply_pending_commands()
        self._update_robot_cells()
        self._process_ingredients(dt)
        self._check_pipeline()
        self._check_order_completions()
        self._sync_all_visuals()
        self._update_ui(dt)

    def _handle_debug_input(self):
        if held_keys["r"]:
            self.reset_game()
        if held_keys["g"]:
            self.gemini_enabled = not self.gemini_enabled
            print(f"[DEBUG] Gemini {'ON' if self.gemini_enabled else 'OFF'}")
        r = self.robots[0]
        speed = 8
        if held_keys["w"]:
            self.physics.apply_force(r.body_id, (speed, 0, 0))
        if held_keys["s"]:
            self.physics.apply_force(r.body_id, (-speed, 0, 0))
        if held_keys["a"]:
            self.physics.apply_force(r.body_id, (0, 0, -speed))
        if held_keys["d"]:
            self.physics.apply_force(r.body_id, (0, 0, speed))

    def _step_physics(self, dt):
        with self._physics_lock:
            steps = max(1, int(dt / self._physics_step_time))
            for _ in range(min(steps, 8)):
                self.physics.step()

    def _call_gemini_if_ready(self):
        if not self.gemini_enabled or self.agent is None:
            return
        now = time.time()
        if now - self._last_gemini_call < CONFIG["gemini_interval"]:
            return
        self._last_gemini_call = now
        state = self._build_game_state()
        threading.Thread(target=self._gemini_thread, args=(state,), daemon=True).start()

    def _gemini_thread(self, state):
        try:
            commands = self.agent.decide(state)
            if commands:
                with self._physics_lock:
                    self._pending_commands.extend(commands)
        except Exception as e:
            print(f"[Gemini] Thread error: {e}")

    def _apply_pending_commands(self):
        with self._physics_lock:
            cmds = self._pending_commands.copy()
            self._pending_commands.clear()
        for cmd in cmds:
            rid = cmd.get("robot_id", 0)
            action = cmd.get("action", "idle")
            target_cell = cmd.get("target_cell")
            target_station = cmd.get("target_station", cmd.get("station"))
            if rid >= len(self.robots):
                continue
            robot = self.robots[rid]
            robot.action = action
            if action == "goto" and target_cell:
                self._navigate_robot(robot, tuple(target_cell))
            elif action == "pickup" and target_cell:
                self._pickup_ingredient(robot, tuple(target_cell))
            elif action == "deliver" and target_cell:
                self._deliver_to_cell(robot, tuple(target_cell), target_station)
            elif action == "process":
                self._process_at_station(robot, target_station)
            elif action == "assemble":
                self._assemble_plate(robot)
            elif action == "pickup_plate":
                self._pickup_plate(robot)
            elif action == "wait":
                robot._stop()
            else:
                robot._stop()

    def _navigate_robot(self, robot, target_cell):
        if robot.current_cell == target_cell:
            robot._stop()
            return
        path = self.pathfinder.find_path(self.passable, robot.current_cell, target_cell)
        if path and len(path) >= 2:
            next_cell = path[1]
            target_pos = self.arena.cell_center(next_cell[0], next_cell[1])
        else:
            target_pos = self.arena.cell_center(target_cell[0], target_cell[1])
        robot.move_toward(target_pos)

    def _pickup_ingredient(self, robot, target_cell):
        if robot.current_cell != target_cell:
            self._navigate_robot(robot, target_cell)
            return
        if robot.carrying:
            robot._stop()
            return
        for ing in self.ingredients:
            if ing.held_by is None and ing.state == "crudo":
                pos = ing.get_position()
                rpos = robot.get_position()
                dist = ((pos.x - rpos.x)**2 + (pos.z - rpos.z)**2) ** 0.5
                if dist < 3.0:
                    robot.pickup(ing)
                    return

    def _deliver_to_cell(self, robot, target_cell, station_name=None):
        if robot.current_cell != target_cell:
            self._navigate_robot(robot, target_cell)
            return
        if robot.carrying:
            pos = robot.get_position()
            robot.drop_at(pos)
        robot._stop()

    def _process_at_station(self, robot, station_name):
        if not robot.carrying:
            return
        if robot.current_cell in [(1, 1), (1, 2)]:
            ing = robot.carrying
            if not ing.processing:
                ing.processing = True
                ing.process_timer = 2.0
        robot._stop()

    def _assemble_plate(self, robot):
        ensamble_cell = (1, 3)
        if robot.current_cell != ensamble_cell:
            self._navigate_robot(robot, ensamble_cell)
            return
        cortados = [i for i in self.ingredients
                    if i.state == "cortado" and i.held_by is None]
        lechuga = [i for i in cortados if i.ingredient_type == "lechuga"]
        tomate = [i for i in cortados if i.ingredient_type == "tomate"]
        if lechuga and tomate and not self._assembled_ingredients:
            lechuga[0].set_state("plato")
            self._assembled_ingredients.append(lechuga[0])
            self._assembled_ingredients.append(tomate[0])
        robot._stop()

    def _pickup_plate(self, robot):
        ensamble_cell = (1, 3)
        if robot.current_cell != ensamble_cell:
            self._navigate_robot(robot, ensamble_cell)
            return
        platos = [i for i in self.ingredients
                  if i.state == "plato" and i.held_by is None
                  and i.ingredient_type == "lechuga"]
        if platos and not robot.carrying:
            robot.pickup(platos[0])
            return
        robot._stop()

    def _update_robot_cells(self):
        for robot in self.robots:
            pos = robot.get_position()
            robot.current_cell = self.arena.get_cell_at(pos.x, pos.z)

    def _process_ingredients(self, dt):
        for ing in self.ingredients:
            if ing.processing and ing.state == "crudo" and ing.held_by is None:
                ing.process_timer -= dt
                if ing.process_timer <= 0:
                    ing.set_state("cortado")
                    ing.processing = False

    def _check_pipeline(self):
        for robot in self.robots:
            if robot.role == "recolector" and not robot.carrying:
                crudos = [i for i in self.ingredients
                         if i.state == "crudo" and i.held_by is None]
                if crudos and self.order_manager.pending > 0:
                    self._navigate_robot(robot, crudos[0].spawn_cell)
            if robot.role == "cortador" and not robot.carrying:
                for ing in self.ingredients:
                    if ing.held_by is None and ing.state == "crudo":
                        rpos = robot.get_position()
                        ipos = ing.get_position()
                        if ((rpos.x-ipos.x)**2 + (rpos.z-ipos.z)**2)**0.5 < 3.5:
                            robot.pickup(ing)
            if robot.role == "repartidor":
                platos = [i for i in self.ingredients
                         if i.state == "plato" and i.held_by is None]
                if platos and not robot.carrying:
                    rpos = robot.get_position()
                    ppos = platos[0].get_position()
                    if ((rpos.x-ppos.x)**2 + (rpos.z-ppos.z)**2)**0.5 < 3.5:
                        robot.pickup(platos[0])
                if robot.carrying and getattr(robot.carrying, "state", "") == "plato":
                    self._navigate_robot(robot, (3, 3))

    def _check_order_completions(self):
        entregada = (3, 3)
        for robot in self.robots:
            if robot.current_cell == entregada and robot.carrying:
                if getattr(robot.carrying, "state", "") == "plato":
                    robot.drop_at(robot.get_position())
                    self.order_manager.complete()
                    self.scoreboard.update(self.order_manager.score,
                                          self.order_manager.completed)
                    print(f"[Order] +150pts | completadas={self.order_manager.completed}")
                    for ing in self.ingredients:
                        ing.reset()
                    self._assembled_ingredients = []
                    self.order_manager.spawn()

    def _sync_all_visuals(self):
        with self._physics_lock:
            for robot in self.robots:
                robot.sync_visual()
            for ing in self.ingredients:
                ing.sync_visual()

    def _update_ui(self, dt):
        robot_states = []
        role_colors_map = {
            "recolector":  [0.20, 0.50, 1.0],
            "cortador":    [1.0, 0.25, 0.25],
            "ensamblador": [0.20, 0.85, 0.25],
            "repartidor":  [1.0, 0.85, 0.15],
        }
        for r in self.robots:
            robot_states.append({
                "cell": r.current_cell,
                "action": r.action,
                "carrying": r.carrying.ingredient_type if r.carrying else None,
                "color": color.rgb(*role_colors_map.get(r.role, [1, 1, 1])),
                "role": r.role,
            })
        stations = {
            "corte_1": {"cell": [1, 1], "processing": any(
                i.processing for i in self.ingredients), "has_plate": False, "collected": []},
            "corte_2": {"cell": [1, 2], "processing": False, "has_plate": False, "collected": []},
            "ensamblaje": {"cell": [1, 3], "processing": False,
                "has_plate": bool(self._assembled_ingredients),
                "collected": [i.ingredient_type for i in self.ingredients
                             if i.state == "cortado" and i.held_by is None]},
        }
        self.hud.update(
            fps=int(1 / max(dt, 0.001)),
            score=self.order_manager.score,
            pending=self.order_manager.pending,
            completed=self.order_manager.completed,
            gemini_enabled=self.gemini_enabled,
            robots=robot_states,
            stations=stations,
        )

    def _build_game_state(self):
        with self._physics_lock:
            robot_states = []
            for r in self.robots:
                robot_states.append({
                    "id": r.robot_id, "role": r.role,
                    "cell": list(r.current_cell), "action": r.action,
                    "carrying": r.carrying.ingredient_type if r.carrying else None,
                })
            ing_states = []
            for ing in self.ingredients:
                cell = ing.spawn_cell if ing.held_by else self.arena.get_cell_at(
                    ing.get_position().x, ing.get_position().z)
                ing_states.append({
                    "type": ing.ingredient_type, "state": ing.state,
                    "cell": list(cell),
                    "held_by": ing.held_by.robot_id if ing.held_by else None,
                })
            stations = {
                "corte_1": {"cell": [1, 1], "processing": any(
                    i.processing and i.held_by is None for i in self.ingredients)},
                "corte_2": {"cell": [1, 2], "processing": False},
                "ensamblaje": {"cell": [1, 3],
                    "has_plate": bool(self._assembled_ingredients),
                    "collected": [i.ingredient_type for i in self.ingredients
                                 if i.state == "cortado" and i.held_by is None]},
                "entrega": {"cell": [3, 3]},
            }
        return {
            "orders": self.order_manager.get_state(),
            "score": self.order_manager.score,
            "robots": robot_states,
            "ingredients": ing_states,
            "stations": stations,
        }

    def reset_game(self):
        self.order_manager.reset()
        self.order_manager.spawn()
        self._assembled_ingredients = []
        for robot in self.robots:
            robot.reset()
        for ing in self.ingredients:
            ing.reset()
        self.scoreboard.update(0, 0)
        print("[RESET] Juego reiniciado")

    def run(self):
        self.app.run()


if __name__ == "__main__":
    game = RoboKitchen()
    game.run()
