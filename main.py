"""
RoboKitchen — MLH Hackathon 2026 | RoBorregos | Tec de Monterrey
Overcooked-style: 4 robots pipeline preparando ensaladas en arena 5x5.
Stack: Ursina (quads texturizados) + PyBullet (fisica) + Gemini API (IA)
"""

import threading
import time
import sys
import json

try:
    from ursina import (Ursina, Entity, Vec3, color, time as utime,
                        held_keys, Text, camera, window, clamp,
                        AmbientLight, DirectionalLight, destroy, Texture)
except ImportError:
    print("[ERROR] Ursina no instalado. pip install ursina")
    sys.exit(1)

try:
    from PIL import Image, ImageDraw
except ImportError:
    print("[ERROR] Pillow no instalado. pip install pillow")
    sys.exit(1)

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    print("[ERROR] PyBullet no instalado. pip install pybullet")
    sys.exit(1)


# ══════════════════════════════════════════════════════════════════════════════
#  PHYSICS WORLD
# ══════════════════════════════════════════════════════════════════════════════

class PhysicsWorld:
    GRAVITY = (0, -9.81, 0)
    STEP_TIME = 1.0 / 240.0

    def __init__(self):
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(*self.GRAVITY, physicsClientId=self.client)
        p.setTimeStep(self.STEP_TIME, physicsClientId=self.client)
        p.setPhysicsEngineParameter(numSolverIterations=50, numSubSteps=4,
                                     physicsClientId=self.client)
        p.loadURDF("plane.urdf", physicsClientId=self.client)
        print(f"[PhysicsWorld] client={self.client}")

    def step(self):
        p.stepSimulation(physicsClientId=self.client)

    def create_box(self, half, mass, pos, friction=0.8, restitution=0.3,
                   linear_damping=0.1, angular_damping=0.1):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.client)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.client)
        bid = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=col,
                                baseVisualShapeIndex=vis, basePosition=pos,
                                physicsClientId=self.client)
        p.changeDynamics(bid, -1, lateralFriction=friction, restitution=restitution,
                         linearDamping=linear_damping, angularDamping=angular_damping,
                         physicsClientId=self.client)
        return bid

    def create_sphere(self, radius, mass, pos, friction=0.5, restitution=0.7):
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=radius, physicsClientId=self.client)
        bid = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=col,
                                basePosition=pos, physicsClientId=self.client)
        p.changeDynamics(bid, -1, lateralFriction=friction, restitution=restitution,
                         linearDamping=0.05, angularDamping=0.05,
                         physicsClientId=self.client)
        return bid

    def get_pos(self, bid):
        pos, _ = p.getBasePositionAndOrientation(bid, physicsClientId=self.client)
        return pos

    def get_vel(self, bid):
        lin, _ = p.getBaseVelocity(bid, physicsClientId=self.client)
        return lin

    def apply_force(self, bid, force):
        p.applyExternalForce(bid, -1, forceObj=force, posObj=(0,0,0),
                             flags=p.WORLD_FRAME, physicsClientId=self.client)

    def reset_body(self, bid, pos):
        p.resetBasePositionAndOrientation(bid, posObj=pos, ornObj=(0,0,0,1),
                                          physicsClientId=self.client)
        p.resetBaseVelocity(bid, linearVelocity=(0,0,0), angularVelocity=(0,0,0),
                            physicsClientId=self.client)


# ══════════════════════════════════════════════════════════════════════════════
#  GEMINI AGENT
# ══════════════════════════════════════════════════════════════════════════════

class GeminiAgent:
    MODEL = "gemini-2.0-flash"
    SYSTEM = (
        "Eres el chef supervisor de RoboKitchen, cocina robotica con 4 robots en pipeline. "
        "Preparan ENSALADAS: lechuga + tomate deben ser CORTADOS (2s), luego ENSAMBLADOS en PLATO, luego ENTREGADOS.\n\n"
        "ROBOTS: 0=RECOLECTOR(rapido), 1=CORTADOR(medio), 2=ENSAMBLADOR(medio), 3=REPARTIDOR(rapido).\n"
        "PIPELINE: ALMACEN->RECOLECTOR recoge->CORTE->CORTADOR rebana 2s->ENSAMBLAJE->ENSAMBLADOR arma->PLATO->REPARTIDOR entrega.\n"
        "ESTACIONES: tomate(1,-2), lechuga(2,-2), corte(-2,-2), ensamblaje(-2,-1), entrega(2,1).\n"
        "HAY PARED en x=0 de z=-2.5 a z=0. Rodear por z>0.\n"
        "RESPONDE SOLO array JSON: [{\"robot_id\":0,\"action\":\"pickup\",\"target\":[x,z]},...]\n"
        "Acciones: pickup(nombre), deliver(estacion), process, assemble, pickup_plate, goto(x,z), wait, idle"
    )

    def __init__(self):
        import os
        from dotenv import load_dotenv
        load_dotenv()
        import google.generativeai as genai
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise EnvironmentError("GEMINI_API_KEY no en .env")
        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel(model_name=self.MODEL, system_instruction=self.SYSTEM)
        self.calls = 0
        print("[GeminiAgent] OK")

    def decide(self, state):
        self.calls += 1
        prompt = f"=== ESTADO #{self.calls} ===\n{json.dumps(state, indent=2)}\n\nDecide acciones para los 4 robots. Responde JSON array."
        try:
            import google.generativeai as genai
            resp = self.model.generate_content(prompt, generation_config=genai.GenerationConfig(
                temperature=0.2, max_output_tokens=300))
            return self._parse(resp.text)
        except Exception as e:
            print(f"[Gemini] Error #{self.calls}: {e}")
            return self._fallback(state)

    def _parse(self, text):
        text = text.strip()
        if text.startswith("```"):
            parts = text.split("```")
            text = parts[1] if len(parts) >= 2 else text
            if text.startswith("json"): text = text[4:]
        text = text.strip()
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            import re
            m = re.search(r"\[.*\]", text, re.DOTALL)
            if m: data = json.loads(m.group())
            else: return []
        return [{"robot_id": int(c.get("robot_id",0)), "action": c.get("action","idle"),
                 "target": c.get("target"), "station": c.get("station", c.get("target"))}
                for c in data if isinstance(c, dict)]

    def _fallback(self, state):
        cmds = []
        robots = {r["id"]: r for r in state.get("robots", [])}
        pending = state.get("orders", {}).get("pending", 0)
        if pending <= 0:
            return [{"robot_id": i, "action": "idle"} for i in range(4)]

        r0 = robots.get(0, {})
        r1 = robots.get(1, {})
        r2 = robots.get(2, {})
        r3 = robots.get(3, {})

        crudos = [i for i in state.get("ingredients", []) if i.get("state") == "crudo" and not i.get("held_by")]
        if crudos and not r0.get("carrying"):
            p = crudos[0]["pos"]
            cmds.append({"robot_id": 0, "action": "pickup", "target": [p[0], p[-1]]})
        elif r0.get("carrying"):
            cmds.append({"robot_id": 0, "action": "deliver", "target": "corte"})
        else:
            cmds.append({"robot_id": 0, "action": "idle"})

        if r1.get("carrying"):
            cmds.append({"robot_id": 1, "action": "process"})
        else:
            cmds.append({"robot_id": 1, "action": "idle"})

        cmds.append({"robot_id": 2, "action": "wait"})
        cmds.append({"robot_id": 3, "action": "wait"})
        return cmds


# ══════════════════════════════════════════════════════════════════════════════
#  ORDER MANAGER
# ══════════════════════════════════════════════════════════════════════════════

class OrderManager:
    def __init__(self, points=150, max_pend=5):
        self.pending = 0
        self.completed = 0
        self.score = 0
        self.points = points
        self.max_pend = max_pend

    def spawn(self):
        if self.pending < self.max_pend:
            self.pending += 1
            return True
        return False

    def complete(self):
        if self.pending > 0: self.pending -= 1
        self.completed += 1
        self.score += self.points

    def get_state(self):
        return {"pending": self.pending, "completed": self.completed, "score": self.score}

    def reset(self):
        self.pending = self.completed = self.score = 0


# ══════════════════════════════════════════════════════════════════════════════
#  TEXTURE GENERATOR
# ══════════════════════════════════════════════════════════════════════════════

def gen_grid_tex(bg_rgba, line_rgba, grid_size=5, res=1024):
    img = Image.new("RGBA", (res, res), bg_rgba)
    draw = ImageDraw.Draw(img)
    lw = 6
    for i in range(grid_size + 1):
        pos = int((i / grid_size) * (res - lw / 2))
        draw.line((0, pos, res, pos), fill=line_rgba, width=lw)
        draw.line((pos, 0, pos, res), fill=line_rgba, width=lw)
    return Texture(img)


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN GAME
# ══════════════════════════════════════════════════════════════════════════════

class RoboKitchen:
    ARENA_HALF = 2.5
    WALL_H = 5.0
    FLOOR_Y = -2.5
    OBJ_Y = -1.75

    STATIONS = {
        "almacen_tomate":  Vec3(1.0, -1.75, -2.0),
        "almacen_lechuga": Vec3(2.0, -1.75, -2.0),
        "corte":           Vec3(-2.0, -1.75, -2.0),
        "ensamblaje":      Vec3(-2.0, -1.75, -1.0),
        "entrega":         Vec3(2.0, -1.75, 1.0),
    }

    ROBOT_STARTS = [
        Vec3(1.5, -1.5, -1.5),
        Vec3(-1.5, -1.5, -1.5),
        Vec3(-1.5, -1.5, -1.0),
        Vec3(1.5, -1.5, 1.0),
    ]

    ROLE_COLORS = {
        "recolector":  color.rgb(50, 120, 240),
        "cortador":    color.rgb(240, 60, 60),
        "ensamblador": color.rgb(50, 210, 60),
        "repartidor":  color.rgb(240, 210, 30),
    }

    ROLE_SPEEDS = {"recolector": 10, "cortador": 7, "ensamblador": 7, "repartidor": 10}
    ROLE_FORCES = {"recolector": 25, "cortador": 18, "ensamblador": 18, "repartidor": 25}

    def __init__(self):
        self.game_active = True
        self.gemini_on = True
        self._lock = threading.Lock()
        self._last_gemini = 0.0
        self._cmds = []
        self.assembled = []

        self.physics = PhysicsWorld()
        self._build_scene()
        self._build_physics_walls()
        self._spawn_ingredients()
        self._spawn_robots()

        try: self.gemini = GeminiAgent()
        except Exception as e: print(f"[Gemini] Off: {e}"); self.gemini_on = False; self.gemini = None

        self.orders = OrderManager()
        self.orders.spawn()
        self._build_ui()

        print("\n[RoboKitchen] OK — WASD/flechas=orbita  R=reset  G=gemini  ESC=salir")
        if self.gemini_on: print(f"  Pedidos pendientes: {self.orders.pending}")

    # ── SCENE ──────────────────────────────────────────────────────────

    def _build_scene(self):
        window.color = color.dark_gray

        floor_tex = gen_grid_tex((35, 38, 48, 255), (220, 220, 235, 200))
        wall_tex = gen_grid_tex((130, 155, 185, 255), (210, 220, 235, 200))

        h = self.ARENA_HALF
        faces = [
            {"pos": (0, 0, h),  "rot": (0, 180, 0),  "eje": "z", "dir": 1},
            {"pos": (0, 0, -h), "rot": (0, 0, 0),    "eje": "z", "dir": -1},
            {"pos": (h, 0, 0),  "rot": (0, 90, 0),   "eje": "x", "dir": 1},
            {"pos": (-h, 0, 0), "rot": (0, -90, 0),  "eje": "x", "dir": -1},
        ]
        self.walls = []
        for f in faces:
            w = Entity(model="quad", scale=(self.ARENA_HALF*2, self.WALL_H),
                       position=f["pos"], rotation=f["rot"], texture=wall_tex,
                       double_sided=True)
            w.eje, w.dir = f["eje"], f["dir"]
            self.walls.append(w)

        Entity(model="quad", scale=(self.ARENA_HALF*2, self.ARENA_HALF*2),
               position=(0, self.FLOOR_Y, 0), rotation=(-90, 0, 0),
               texture=floor_tex, double_sided=True)

        self._station_cubes()

        self.internal_wall = Entity(model="cube", color=color.gray,
                                     position=(0, 0, -1.25), scale=(0.1, 5, 2.5),
                                     unlit=True, edge_color=color.black, edge_width=2)

        self.pivot = Entity()
        camera.parent = self.pivot
        camera.position = (0, 0, -18)
        self.pivot.rotation_x, self.pivot.rotation_y = 35, 45
        AmbientLight(color=color.rgba(190, 190, 210, 255))
        dl = DirectionalLight(); dl.look_at(Vec3(0.5, -1, -0.5))

    def _station_cubes(self):
        stations = [
            (self.STATIONS["almacen_tomate"],  color.red,    "TOMATE"),
            (self.STATIONS["almacen_lechuga"], color.green,  "LECHUGA"),
            (self.STATIONS["corte"],           color.yellow, "CORTE"),
            (self.STATIONS["ensamblaje"],      color.brown,  "ENSAMBLE"),
            (self.STATIONS["entrega"],         color.azure,  "ENTREGA"),
        ]
        for pos, clr, label in stations:
            Entity(model="cube", color=clr, position=pos, scale=1,
                   unlit=True, edge_color=color.black, edge_width=3)
            Text(text=label, position=pos + Vec3(0, 0.8, 0),
                 origin=(0, 0), scale=0.8, color=color.white, billboard=True)

    def _build_physics_walls(self):
        h = self.ARENA_HALF
        wh = self.WALL_H
        hw = wh / 2
        for wx, wz in [(h + 0.15, 0), (-(h + 0.15), 0), (0, h + 0.15), (0, -(h + 0.15))]:
            hx, hz = (0.15, h + 0.15) if wx == 0 else (h + 0.15, 0.15)
            self.physics.create_box(half=(hx, wh/2, hz), mass=0, pos=(wx, hw, wz))
        self.physics.create_box(half=(0.05, 2.5, 1.25), mass=0, pos=(0, 2.5, -1.25))

    # ── INGREDIENTS ────────────────────────────────────────────────────

    def _spawn_ingredients(self):
        self.ingredients = []
        for itype, spos in [("lechuga", self.STATIONS["almacen_lechuga"]),
                             ("tomate", self.STATIONS["almacen_tomate"])]:
            ing = Ingredient(self.physics, itype, spos)
            self.ingredients.append(ing)

    # ── ROBOTS ─────────────────────────────────────────────────────────

    def _spawn_robots(self):
        self.robots = []
        roles = ["recolector", "cortador", "ensamblador", "repartidor"]
        for i, role in enumerate(roles):
            r = Robot(self.physics, i, role, self.ROBOT_STARTS[i])
            self.robots.append(r)

    # ── UI ──────────────────────────────────────────────────────────────

    def _build_ui(self):
        legend = ("<red>Rojo:<default> Tomate\n<green>Verde:<default> Lechuga\n"
                  "<yellow>Amarillo:<default> Corte\n<brown>Cafe:<default> Ensamblaje\n"
                  "<azure>Azul:<default> Entrega\nGris: Pared")
        Text(text=legend, position=(0.55, 0.4), origin=(-0.5, 0.5), scale=0.85, background=True)

        cols = ["<blue>R0 REC", "<red>R1 COR", "<green>R2 ENS", "<yellow>R3 REP"]
        self.robot_ui_lines = []
        for i, label in enumerate(cols):
            t = Text(text=label, position=(-0.75, 0.45 - i * 0.055),
                     origin=(-0.5, 0.5), scale=0.8)
            self.robot_ui_lines.append(t)

        self.score_txt = Text(text="SCORE: 0 | PEDIDOS: 1", position=(-0.75, 0.22),
                              origin=(-0.5, 0.5), scale=0.9, color=color.yellow)
        self.gemini_txt = Text(text="Gemini: ON", position=(-0.75, 0.16),
                               origin=(-0.5, 0.5), scale=0.8, color=color.green)

    # ── GAME LOOP ──────────────────────────────────────────────────────

    def update(self):
        if not self.game_active:
            return
        dt = utime.dt
        if dt <= 0 or dt > 0.1: return

        self._orbit(dt)
        self._wall_xray()
        self._input()
        self._physics(dt)
        self._gemini_tick()
        self._apply_cmds()
        self._process_ingredients(dt)
        self._pipeline()
        self._check_deliveries()
        self._sync_visuals()
        self._ui()

    def _orbit(self, dt):
        sp = 70 * dt
        self.pivot.rotation_y += (held_keys["d"] - held_keys["a"] +
                                   held_keys["right arrow"] - held_keys["left arrow"]) * sp
        self.pivot.rotation_x += (held_keys["w"] - held_keys["s"] +
                                   held_keys["up arrow"] - held_keys["down arrow"]) * sp
        self.pivot.rotation_x = clamp(self.pivot.rotation_x, 10, 85)

    def _wall_xray(self):
        cp = camera.world_position
        for w in self.walls:
            if w.eje == "x":
                w.enabled = not ((w.dir == 1 and cp.x > w.position.x) or
                                 (w.dir == -1 and cp.x < w.position.x))
            elif w.eje == "z":
                w.enabled = not ((w.dir == 1 and cp.z > w.position.z) or
                                 (w.dir == -1 and cp.z < w.position.z))

    def _input(self):
        if held_keys["r"]: self.reset()
        if held_keys["g"]:
            self.gemini_on = not self.gemini_on
            print(f"[Gemini] {'ON' if self.gemini_on else 'OFF'}")

    def _physics(self, dt):
        with self._lock:
            for _ in range(min(max(1, int(dt * 240)), 8)):
                self.physics.step()

    def _gemini_tick(self):
        if not self.gemini_on or not self.gemini: return
        now = time.time()
        if now - self._last_gemini < 3.0: return
        self._last_gemini = now
        state = self._state()
        threading.Thread(target=self._gemini_thread, args=(state,), daemon=True).start()

    def _gemini_thread(self, state):
        try:
            cmds = self.gemini.decide(state)
            if cmds:
                with self._lock: self._cmds.extend(cmds)
        except Exception as e: print(f"[Gemini] Thread: {e}")

    def _apply_cmds(self):
        with self._lock:
            cmds = self._cmds.copy(); self._cmds.clear()
        for c in cmds:
            rid = c.get("robot_id", 0)
            act = c.get("action", "idle")
            tgt = c.get("target")
            stn = c.get("station", tgt)
            if rid >= len(self.robots): continue
            r = self.robots[rid]
            r.action = act
            if act in ("goto", "pickup", "deliver"):
                self._nav(r, tgt, act, stn)
            elif act == "process":
                if r.carrying and r.current_station == "corte":
                    r.carrying.processing = True; r.carrying.process_timer = 2.0
            elif act == "assemble":
                self._try_assemble(r)
            elif act == "pickup_plate":
                self._try_pickup_plate(r)
            else:
                r.stop()

    def _nav(self, robot, target, action, station_name=None):
        tpos = self._resolve_target(target, station_name)
        if tpos is None:
            robot.stop(); return
        dist = (robot.pos() - tpos).length()
        if dist < 1.0:
            if action == "pickup":
                self._try_pickup(robot)
            elif action == "deliver":
                if robot.carrying: robot.drop(robot.pos())
            robot.stop()
        else:
            dx = tpos.x - robot.pos().x
            dz = tpos.z - robot.pos().z
            d = (dx*dx + dz*dz) ** 0.5
            if d > 0.001:
                f = self.ROLE_FORCES[robot.role]
                self.physics.apply_force(robot.body, (dx/d*f, 0, dz/d*f))
            self._clamp(robot)

    def _resolve_target(self, target, station_name):
        if isinstance(target, (list, tuple)) and len(target) >= 2:
            return Vec3(target[0], self.OBJ_Y, target[1])
        if isinstance(target, str):
            st = self.STATIONS.get(target) or self.STATIONS.get(station_name)
            return st
        return None

    def _try_pickup(self, robot):
        if robot.carrying: return
        for ing in self.ingredients:
            if ing.held or ing.state != "crudo": continue
            d = (robot.pos() - ing.pos()).length()
            if d < 1.5:
                robot.pickup(ing); return

    def _try_assemble(self, robot):
        if robot.current_station != "ensamblaje":
            self._nav(robot, self.STATIONS["ensamblaje"], "goto"); return
        cortados = [i for i in self.ingredients if i.state == "cortado" and not i.held]
        lech = [i for i in cortados if i.itype == "lechuga"]
        tom = [i for i in cortados if i.itype == "tomate"]
        if lech and tom and not self.assembled:
            lech[0].set_state("plato")
            self.assembled = [lech[0], tom[0]]

    def _try_pickup_plate(self, robot):
        if robot.current_station != "ensamblaje":
            self._nav(robot, self.STATIONS["ensamblaje"], "goto"); return
        platos = [i for i in self.ingredients if i.state == "plato" and not i.held]
        if platos and not robot.carrying: robot.pickup(platos[0])

    def _clamp(self, robot):
        v = self.physics.get_vel(robot.body)
        sp = (v[0]**2 + v[2]**2) ** 0.5
        mx = self.ROLE_SPEEDS[robot.role]
        if sp > mx:
            f = mx / sp
            p.resetBaseVelocity(robot.body, linearVelocity=(v[0]*f, v[1], v[2]*f),
                                angularVelocity=(0,0,0), physicsClientId=self.physics.client)

    def _process_ingredients(self, dt):
        for ing in self.ingredients:
            if ing.processing and ing.state == "crudo" and not ing.held:
                ing.process_timer -= dt
                if ing.process_timer <= 0:
                    ing.set_state("cortado"); ing.processing = False

    def _pipeline(self):
        for r in self.robots:
            if r.role == "recolector" and not r.carrying:
                crudos = [i for i in self.ingredients if i.state == "crudo" and not i.held]
                if crudos and self.orders.pending > 0:
                    self._nav(r, crudos[0].spawn, "goto")
            if r.role == "cortador" and not r.carrying and r.current_station == "corte":
                for ing in self.ingredients:
                    if not ing.held and ing.state == "crudo":
                        d = (r.pos() - ing.pos()).length()
                        if d < 1.5: r.pickup(ing)
            if r.role == "repartidor":
                platos = [i for i in self.ingredients if i.state == "plato" and not i.held]
                if platos and not r.carrying:
                    d = (r.pos() - platos[0].pos()).length()
                    if d < 2: r.pickup(platos[0])
                if r.carrying and getattr(r.carrying, "state", "") == "plato":
                    self._nav(r, self.STATIONS["entrega"], "deliver")

    def _check_deliveries(self):
        entrega = self.STATIONS["entrega"]
        for r in self.robots:
            if r.carrying and getattr(r.carrying, "state", "") == "plato":
                d = (r.pos() - entrega).length()
                if d < 1.5:
                    r.drop(r.pos())
                    self.orders.complete()
                    print(f"[Order] +150pts | total={self.orders.completed} | score={self.orders.score}")
                    for ing in self.ingredients: ing.reset()
                    self.assembled = []
                    self.orders.spawn()

    def _update_stations(self):
        for r in self.robots:
            p = r.pos()
            if abs(p.x - (-2.0)) < 0.6 and abs(p.z - (-2.0)) < 0.6: r.current_station = "corte"
            elif abs(p.x - (-2.0)) < 0.6 and abs(p.z - (-1.0)) < 0.6: r.current_station = "ensamblaje"
            elif abs(p.x - 2.0) < 0.6 and abs(p.z - 1.0) < 0.6: r.current_station = "entrega"
            elif p.x > 0 and p.z < -1.0: r.current_station = "almacen"
            else: r.current_station = "pasillo"

    def _sync_visuals(self):
        self._update_stations()
        with self._lock:
            for r in self.robots: r.sync()
            for i in self.ingredients: i.sync()

    def _ui(self):
        for i, r in enumerate(self.robots):
            c = "+" if r.carrying else "-"
            a = r.action[:6] if r.action else "idle"
            st = r.current_station[:6] if r.current_station else "?"
            self.robot_ui_lines[i].text = f"R{i} {r.role[:3].upper()} | {st} | {a} [{c}]"
        self.score_txt.text = f"SCORE: {self.orders.score} | PEDIDOS: {self.orders.pending}"
        gc = color.green if self.gemini_on else color.red
        self.gemini_txt.text = f"Gemini: {'ON' if self.gemini_on else 'OFF'}"; self.gemini_txt.color = gc

    def _state(self):
        with self._lock:
            rs = []
            for r in self.robots:
                p = r.pos()
                rs.append({"id": r.rid, "role": r.role, "pos": [round(p.x,2), round(p.z,2)],
                           "carrying": r.carrying.itype if r.carrying else None,
                           "station": r.current_station, "action": r.action})
            igs = []
            for i in self.ingredients:
                p = i.pos()
                igs.append({"type": i.itype, "state": i.state,
                            "pos": [round(p.x,2), round(p.z,2)],
                            "held_by": i.held.rid if i.held else None})
        return {"orders": self.orders.get_state(), "robots": rs, "ingredients": igs,
                "score": self.orders.score}

    def reset(self):
        self.orders.reset(); self.orders.spawn()
        self.assembled = []
        for r in self.robots: r.reset()
        for i in self.ingredients: i.reset()
        print("[RESET]")


# ══════════════════════════════════════════════════════════════════════════════
#  ROBOT
# ══════════════════════════════════════════════════════════════════════════════

class Robot:
    def __init__(self, physics, rid, role, start):
        self.physics = physics; self.rid = rid; self.role = role
        self.start = start; self.action = "idle"; self.carrying = None
        self.current_station = "pasillo"

        self.body = physics.create_box(half=(0.4, 0.35, 0.5), mass=3.0,
                                       pos=(start.x, start.y, start.z),
                                       linear_damping=0.5, angular_damping=0.9)
        clr = RoboKitchen.ROLE_COLORS[role]
        self.vis = Entity(model="cube", color=clr, scale=(0.8, 0.7, 1.0),
                          position=start, unlit=True, edge_color=color.black, edge_width=2)
        self.dot = Entity(parent=self.vis, model="quad", color=color.yellow,
                          scale=(0.3, 0.3, 1), position=Vec3(0, 0.9, 0),
                          billboard=True, enabled=False)

    def pos(self):
        p = self.physics.get_pos(self.body); return Vec3(p[0], p[1], p[2])

    def sync(self):
        p = self.physics.get_pos(self.body)
        self.vis.position = Vec3(p[0], p[1], p[2])
        self.dot.enabled = self.carrying is not None

    def move(self, target):
        dx = target.x - self.pos().x; dz = target.z - self.pos().z
        d = (dx*dx + dz*dz)**0.5
        if d > 0.1: f = RoboKitchen.ROLE_FORCES[self.role]; self.physics.apply_force(self.body, (dx/d*f, 0, dz/d*f))

    def stop(self):
        self.physics.reset_body(self.body, (self.pos().x, self.pos().y, self.pos().z))

    def pickup(self, ing):
        self.carrying = ing; ing.held = self; self.action = "carrying"
        ing.vis.parent = self.vis; ing.vis.position = Vec3(0, 1.2, 0)

    def drop(self, pos):
        if self.carrying:
            self.carrying.vis.parent = None
            self.carrying.held = None
            self.physics.reset_body(self.carrying.body, (pos.x, pos.y + 0.3, pos.z))
            self.carrying.vis.position = Vec3(pos.x, pos.y + 0.3, pos.z)
            self.carrying = None
        self.action = "idle"

    def reset(self):
        if self.carrying: self.carrying.vis.parent = None; self.carrying.held = None
        self.carrying = None; self.action = "idle"; self.current_station = "pasillo"
        self.physics.reset_body(self.body, (self.start.x, self.start.y, self.start.z))
        self.vis.position = self.start


# ══════════════════════════════════════════════════════════════════════════════
#  INGREDIENT
# ══════════════════════════════════════════════════════════════════════════════

class Ingredient:
    COLORS = {
        "lechuga": {"crudo": color.rgb(60, 190, 60), "cortado": color.rgb(30, 240, 30),
                    "plato": color.rgb(255, 210, 60)},
        "tomate":  {"crudo": color.rgb(220, 50, 50), "cortado": color.rgb(255, 30, 30),
                    "plato": color.rgb(255, 210, 60)},
    }

    def __init__(self, physics, itype, spawn_world):
        self.physics = physics; self.itype = itype
        self.state = "crudo"; self.spawn = spawn_world
        self.held = None; self.processing = False; self.process_timer = 0.0

        self.body = physics.create_sphere(0.25, 1.0, (spawn_world.x, spawn_world.y, spawn_world.z))
        self.vis = Entity(model="sphere", color=self.COLORS[itype]["crudo"],
                          scale=0.5, position=spawn_world, unlit=True,
                          edge_color=color.black, edge_width=2)

    def pos(self):
        p = self.physics.get_pos(self.body); return Vec3(p[0], p[1], p[2])

    def sync(self):
        if self.held: return
        p = self.physics.get_pos(self.body); self.vis.position = Vec3(p[0], p[1], p[2])

    def set_state(self, s):
        self.state = s
        self.vis.color = self.COLORS[self.itype][s]
        self.vis.scale = 0.8 if s == "plato" else (0.5 if s == "cortado" else 0.5)
        if s == "plato": self.vis.model = "cube"

    def reset(self):
        self.state = "crudo"; self.held = None; self.processing = False
        self.process_timer = 0.0; self.vis.parent = None
        self.vis.model = "sphere"; self.vis.scale = 0.5
        self.vis.color = self.COLORS[self.itype]["crudo"]
        self.physics.reset_body(self.body, (self.spawn.x, self.spawn.y, self.spawn.z))
        self.vis.position = self.spawn


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = Ursina(title="RoboKitchen — MLH Hackathon 2026 | RoBorregos",
                 size=(1280, 720), development_mode=True)
    window.fps_counter.enabled = False
    window.exit_button.visible = False
    game = RoboKitchen()
    app.run()
