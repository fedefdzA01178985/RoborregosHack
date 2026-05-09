"""
RoboKitchen — MLH Hackathon 2026 | RoBorregos | Tec de Monterrey
Overcooked 3D: 4 robots pipeline preparando ensaladas.
Ursina (visual) + PyBullet (fisica) + Gemini (randomizador de escenarios).
"""

import threading, time, json, sys, random, math

# ── Dependencias ─────────────────────────────────────────────────────────────
try:
    from PIL import Image, ImageDraw
except ImportError:
    print("[ERROR] Pillow no instalado. pip install pillow"); sys.exit(1)

try:
    from ursina import (Ursina, Entity, Vec3, color, time as utime,
                        held_keys, Text, camera, window, clamp,
                        AmbientLight, DirectionalLight, Texture)
except ImportError:
    print("[ERROR] Ursina no instalado. pip install ursina"); sys.exit(1)

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    print("[ERROR] PyBullet no instalado. pip install pybullet"); sys.exit(1)

# ══════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════════════════
ARENA_HALF = 2.5
WALL_H = 5.0
FLOOR_Y = -2.5
OBJ_Y = FLOOR_Y + 0.5
ROBOT_SPEEDS  = {"recolector": 10, "cortador": 7, "ensamblador": 7, "repartidor": 10}
ROBOT_FORCES  = {"recolector": 25, "cortador": 18, "ensamblador": 18, "repartidor": 25}
ROBOT_COLORS  = {
    "recolector":  color.rgb(50, 120, 240),
    "cortador":    color.rgb(240, 60, 60),
    "ensamblador": color.rgb(50, 210, 60),
    "repartidor":  color.rgb(240, 210, 30),
}
ROBOT_Y = FLOOR_Y + 0.35
ROBOT_STARTS  = [
    Vec3(2.0,  ROBOT_Y, 2.2),
    Vec3(0.7,  ROBOT_Y, 2.2),
    Vec3(-0.7, ROBOT_Y, 2.2),
    Vec3(-2.0, ROBOT_Y, 2.2),
]
# STATIONS es mutable — Scenario lo actualiza
STATIONS = {
    "almacen_tomate":  Vec3(1.0, OBJ_Y, -2.0),
    "almacen_lechuga": Vec3(2.0, OBJ_Y, -2.0),
    "corte":           Vec3(-2.0, OBJ_Y, -2.0),
    "ensamblaje":      Vec3(-2.0, OBJ_Y, -1.0),
    "platos":          Vec3(-2.0, OBJ_Y, 0.0),
    "entrega":         Vec3(2.0, OBJ_Y, 1.0),
}
STATION_DEFAULTS = dict(STATIONS)
INGREDIENT_COLORS = {
    "lechuga": {"crudo": color.rgb(60, 190, 60), "cortado": color.rgb(30, 240, 30),
                "plato": color.rgb(255, 210, 60)},
    "tomate":  {"crudo": color.rgb(220, 50, 50), "cortado": color.rgb(255, 30, 30),
                "plato": color.rgb(255, 210, 60)},
    "plato":   {"crudo": color.rgb(240, 240, 240), "cortado": color.rgb(240, 240, 240),
                "plato": color.rgb(255, 220, 80)},
}

# ══════════════════════════════════════════════════════════════════════════════
#  TEXTURAS
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
#  PHYSICS WORLD
# ══════════════════════════════════════════════════════════════════════════════

class PhysicsWorld:
    STEP = 1.0 / 240.0

    def __init__(self):
        self.cli = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.cli)
        p.setGravity(0, -9.81, 0, physicsClientId=self.cli)
        p.setTimeStep(self.STEP, physicsClientId=self.cli)
        p.setPhysicsEngineParameter(numSolverIterations=50, numSubSteps=4, physicsClientId=self.cli)
        print(f"[Physics] cli={self.cli}")

    def step(self): p.stepSimulation(physicsClientId=self.cli)

    def box(self, half, mass, pos, friction=0.8, ldamp=0.1, adamp=0.1):
        c = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.cli)
        v = p.createVisualShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.cli)
        b = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=c,
                              baseVisualShapeIndex=v, basePosition=pos, physicsClientId=self.cli)
        p.changeDynamics(b, -1, lateralFriction=friction, linearDamping=ldamp,
                         angularDamping=adamp, physicsClientId=self.cli)
        return b

    def sphere(self, r, mass, pos):
        c = p.createCollisionShape(p.GEOM_SPHERE, radius=r, physicsClientId=self.cli)
        b = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=c,
                              basePosition=pos, physicsClientId=self.cli)
        p.changeDynamics(b, -1, lateralFriction=0.5, restitution=0.7,
                         linearDamping=0.05, angularDamping=0.05, physicsClientId=self.cli)
        return b

    def pos(self, b):
        pp, _ = p.getBasePositionAndOrientation(b, physicsClientId=self.cli); return pp

    def vel(self, b):
        v, _ = p.getBaseVelocity(b, physicsClientId=self.cli); return v

    def force(self, b, f):
        p.applyExternalForce(b, -1, forceObj=f, posObj=(0,0,0),
                             flags=p.WORLD_FRAME, physicsClientId=self.cli)

    def reset(self, b, pos):
        p.resetBasePositionAndOrientation(b, posObj=pos, ornObj=(0,0,0,1), physicsClientId=self.cli)
        p.resetBaseVelocity(b, linearVelocity=(0,0,0), angularVelocity=(0,0,0), physicsClientId=self.cli)

# ══════════════════════════════════════════════════════════════════════════════
#  ROBOT
# ══════════════════════════════════════════════════════════════════════════════

class Robot:
    def __init__(self, pw, rid, role, start):
        self.pw = pw; self.rid = rid; self.role = role
        self.start = start; self.action = "idle"; self.carrying = None
        self.carrying_plate = None; self.station = "pasillo"
        # Cuerpo físico
        self.body = pw.box((0.35, 0.35, 0.45), 3.0, (start.x, start.y, start.z),
                           ldamp=0.5, adamp=0.9)
        # Visual principal
        self.vis = Entity(model="cube", color=ROBOT_COLORS[role],
                          scale=(0.7, 0.7, 0.9), position=start,
                          unlit=True, edge_color=color.black, edge_width=2)
        # Ruedas (4 cubos pequeños)
        wcol = color.rgb(40,40,40)
        self.wheels = []
        for wx, wz in [(-0.25,-0.3),(0.25,-0.3),(-0.25,0.3),(0.25,0.3)]:
            w = Entity(parent=self.vis, model="cube", color=wcol, scale=(0.15,0.1,0.15),
                       position=Vec3(wx, -0.45, wz))
            self.wheels.append(w)
        # "Brazo" / indicador
        self.arm = Entity(parent=self.vis, model="cube", color=color.orange,
                          scale=(0.1, 0.4, 0.1), position=Vec3(0, 0.55, 0.3))
        # Dot de carga
        self.dot = Entity(parent=self.vis, model="sphere", color=color.yellow,
                          scale=0.15, position=Vec3(0, 0.9, 0),
                          billboard=True, enabled=False)
        # Timer para pathfinding evasión
        self._avoid_t = 0.0
        self._avoid_dir = 1

    def pos(self): p = self.pw.pos(self.body); return Vec3(p[0], p[1], p[2])

    def sync(self):
        p = self.pw.pos(self.body); self.vis.position = Vec3(p[0], p[1], p[2])
        # Girar hacia la dirección de movimiento
        v = self.pw.vel(self.body)
        if abs(v[0]) > 0.1 or abs(v[2]) > 0.1:
            angle = math.degrees(math.atan2(v[0], v[2]))
            self.vis.rotation_y = angle
        self.dot.enabled = self.carrying is not None or self.carrying_plate is not None

    def move_to(self, target):
        pp = self.pos(); dx = target.x - pp.x; dz = target.z - pp.z
        d = (dx*dx + dz*dz)**0.5
        if d < 0.5: self.stop(); return
        f = ROBOT_FORCES[self.role]
        # Si estamos en modo evasión, desviar
        if self._avoid_t > 0:
            self._avoid_t -= utime.dt
            perp_x = -dz/d * self._avoid_dir
            perp_z = dx/d * self._avoid_dir
            self.pw.force(self.body, ((dx/d*0.5 + perp_x*0.5)*f, 0, (dz/d*0.5 + perp_z*0.5)*f))
        else:
            self.pw.force(self.body, (dx/d*f, 0, dz/d*f))
        self._clamp()

    def stop(self): self.pw.reset(self.body, (self.pos().x, self.pos().y, self.pos().z))

    def _clamp(self):
        v = self.pw.vel(self.body); s = (v[0]**2+v[2]**2)**0.5
        mx = ROBOT_SPEEDS[self.role]
        if s > mx:
            f = mx/s
            p.resetBaseVelocity(self.body, linearVelocity=(v[0]*f, v[1], v[2]*f),
                                angularVelocity=(0,0,0), physicsClientId=self.pw.cli)

    def avoid(self):
        """Llamar cuando detecta obstáculo — gira momentáneamente"""
        self._avoid_t = 0.5
        self._avoid_dir = random.choice([-1, 1])

    def pickup(self, ing):
        self.carrying = ing; ing.held = self; self.action = "carrying"
        ing.vis.parent = self.vis; ing.vis.position = Vec3(0, 1.2, 0)

    def drop(self, wpos):
        if self.carrying:
            self.carrying.vis.parent = None; self.carrying.held = None
            self.pw.reset(self.carrying.body, (wpos.x, wpos.y+0.3, wpos.z))
            self.carrying.vis.position = Vec3(wpos.x, wpos.y+0.3, wpos.z)
            self.carrying = None
        self.action = "idle"

    def reset(self):
        if self.carrying:
            self.carrying.vis.parent = None; self.carrying.held = None; self.carrying = None
        if self.carrying_plate:
            self.carrying_plate.vis.parent = None
            self.carrying_plate.held = None; self.carrying_plate = None
        self.action = "idle"; self.station = "pasillo"
        self._avoid_t = 0.0
        self.pw.reset(self.body, (self.start.x, self.start.y, self.start.z))
        self.vis.position = self.start

# ══════════════════════════════════════════════════════════════════════════════
#  INGREDIENTE
# ══════════════════════════════════════════════════════════════════════════════

class Ingredient:
    def __init__(self, pw, itype, spawn, state="crudo"):
        self.pw = pw; self.itype = itype; self.state = state
        self.spawn = spawn; self.held = None; self.proc = False; self.proc_t = 0.0
        self.body = pw.sphere(0.2, 0, (spawn.x, spawn.y + 0.1, spawn.z))
        clr = INGREDIENT_COLORS[itype].get(state, INGREDIENT_COLORS[itype]["crudo"])
        self.vis = Entity(model="cube" if state == "plato" else "sphere",
                          color=clr, scale=0.8 if state == "plato" else 0.5,
                          position=spawn + Vec3(0, 0.3, 0),
                          unlit=True, edge_color=color.black, edge_width=2)

    def pos(self):
        if self.held: return self.held.pos() + Vec3(0, 0.5, 0)
        return self.spawn + Vec3(0, 0.3, 0)

    def sync(self):
        if not self.held:
            self.pw.reset(self.body, (self.spawn.x, self.spawn.y + 0.1, self.spawn.z))

    def set_state(self, s):
        self.state = s; self.vis.color = INGREDIENT_COLORS[self.itype][s]
        if s == "plato": self.vis.model = "cube"; self.vis.scale = 0.8

    def reset(self):
        self.state = "crudo"; self.held = None; self.proc = False; self.proc_t = 0.0
        self.vis.parent = None; self.vis.model = "sphere"; self.vis.scale = 0.5
        self.vis.color = INGREDIENT_COLORS[self.itype]["crudo"]
        self.vis.position = self.spawn + Vec3(0, 0.3, 0)

# ══════════════════════════════════════════════════════════════════════════════
#  GEMINI AGENT — Scenario Architect
# ══════════════════════════════════════════════════════════════════════════════

class GeminiAgent:
    SYS = (
        "Eres un arquitecto de cocina robotica. Genera escenarios variados para RoboKitchen.\n"
        "Responde SOLO con este JSON exacto:\n"
        "{\n"
        '  "stations": {\n'
        '    "almacen_tomate":  [x, z],\n'
        '    "almacen_lechuga": [x, z],\n'
        '    "corte":           [x, z],\n'
        '    "ensamblaje":      [x, z],\n'
        '    "platos":          [x, z],\n'
        '    "entrega":         [x, z]\n'
        "  },\n"
        '  "obstacles": [\n'
        '    {"x": num, "z": num, "scale": [w, h, d]}, ...\n'
        "  ]\n"
        "}\n"
        "Reglas:\n"
        "- x entre -2 y 2, z entre -2 y 2.\n"
        "- almacenes: x > 0 (zona derecha).\n"
        "- corte/ensamblaje/platos: x < 0 (zona izquierda).\n"
        "- entrega: x > 0, z > 0.\n"
        "- 2-4 obstaculos, scale entre [0.3,0.3,0.3] y [1.0,0.3,1.0].\n"
        "- Deja pasillo central libre (z>0 o z<0)."
    )
    MODEL = "gemini-2.0-flash"

    def __init__(self):
        import os
        from dotenv import load_dotenv
        load_dotenv()
        from google import genai
        key = os.getenv("GEMINI_API_KEY")
        if not key:
            print("[Gemini] GEMINI_API_KEY no en .env — scenario manual activo")
            self._ok = False; self.client = None; return
        try:
            self.client = genai.Client(api_key=key)
            test = self.client.models.generate_content(
                model=self.MODEL,
                contents="Responde: OK",
                config=genai.types.GenerateContentConfig(temperature=0, max_output_tokens=10))
            if "OK" in (test.text or "").upper():
                self._ok = True; print("[Gemini] API OK — scenario architect listo")
            else:
                self._ok = False; print("[Gemini] API respondio pero sin OK")
        except Exception as e:
            self._ok = False
            print(f"[Gemini] API NO disponible ({type(e).__name__}) — scenario manual activo")

    def generate_scenario(self):
        if not self._ok or not self.client:
            return self._default_scenario()
        prompt = self.SYS + "\n\nGenera escenario aleatorio:"
        try:
            r = self.client.models.generate_content(
                model=self.MODEL,
                contents=prompt,
                config=genai.types.GenerateContentConfig(temperature=0.9, max_output_tokens=400))
            data = self._parse(r.text)
            if data: return data
        except Exception as e:
            print(f"[Gemini] Error generando escenario: {type(e).__name__}")
        return self._default_scenario()

    def _parse(self, t):
        t = (t or "").strip()
        if t.startswith("```"):
            p = t.split("```"); t = p[1] if len(p)>=2 else t
            if t.startswith("json"): t = t[4:]
        t = t.strip()
        try:
            d = json.loads(t)
            if "stations" in d and "obstacles" in d:
                return d
        except:
            import re; m = re.search(r"\{.*\}", t, re.DOTALL)
            if m:
                try:
                    d = json.loads(m.group())
                    if "stations" in d and "obstacles" in d:
                        return d
                except: pass
        return None

    def _default_scenario(self):
        return {
            "stations": {
                "almacen_tomate":  [1.0, -2.0],
                "almacen_lechuga": [2.0, -2.0],
                "corte":           [-2.0, -2.0],
                "ensamblaje":      [-2.0, -1.0],
                "platos":          [-2.0, 0.0],
                "entrega":         [2.0, 1.0],
            },
            "obstacles": [
                {"x": 0.0, "z": 1.5, "scale": [0.6, 0.3, 0.6]},
                {"x": -1.0, "z": 1.5, "scale": [0.6, 0.3, 0.6]},
            ]
        }

# ══════════════════════════════════════════════════════════════════════════════
#  SCENARIO — aplica escenarios al mundo
# ══════════════════════════════════════════════════════════════════════════════

class Scenario:
    def __init__(self, pw):
        self.pw = pw
        self.vis_refs = {}   # {"corte": Entity, ...}
        self.text_refs = {}  # {"corte": Text, ...}
        self.body_refs = {}  # {"corte": pybullet_id, ...}
        self.obstacle_vis = []  # lista de Entity
        self.obstacle_body = [] # lista de pybullet_id
        self._current = GeminiAgent()._default_scenario()

    def set_refs(self, vis, texts, bodies):
        self.vis_refs = vis
        self.text_refs = texts
        self.body_refs = bodies

    def apply(self, config):
        self._current = config
        # Actualizar STATIONS global
        for k, v in config.get("stations", {}).items():
            STATIONS[k] = Vec3(v[0], OBJ_Y, v[1])
            # Mover visual
            if k in self.vis_refs:
                self.vis_refs[k].position = Vec3(v[0], OBJ_Y, v[1])
            if k in self.text_refs:
                self.text_refs[k].position = Vec3(v[0], OBJ_Y + 0.9, v[1])
            if k in self.body_refs:
                self.pw.reset(self.body_refs[k], (v[0], OBJ_Y, v[1]))
        # Actualizar ingredientes spawn
        for ing in ingredients:
            st_key = "almacen_lechuga" if ing.itype == "lechuga" else "almacen_tomate"
            if st_key in STATIONS:
                ing.spawn = STATIONS[st_key]
                ing.reset()
        # Actualizar plato spawn
        for pl in plates:
            pl.spawn = STATIONS["platos"]
            pl.reset()
        # Limpiar y crear obstáculos
        self._clear_obstacles()
        for obs in config.get("obstacles", []):
            pos = (obs["x"], OBJ_Y, obs["z"])
            scl = obs.get("scale", [0.5, 0.3, 0.5])
            half = [s/2 for s in scl]
            # Visual
            vis = Entity(model="cube", color=color.gray,
                         position=Vec3(pos[0], pos[1], pos[2]),
                         scale=tuple(scl),
                         unlit=True, edge_color=color.black, edge_width=2)
            self.obstacle_vis.append(vis)
            # Física
            body = self.pw.box(half, 0, pos)
            self.obstacle_body.append(body)

    def _clear_obstacles(self):
        for v in self.obstacle_vis:
            if hasattr(v, 'enabled'): v.enabled = False
            if hasattr(v, 'parent') and v.parent: v.parent = None
        for b in self.obstacle_body:
            p.removeBody(b, physicsClientId=self.pw.cli)
        self.obstacle_vis.clear()
        self.obstacle_body.clear()

    def reset_default(self):
        self.apply(GeminiAgent()._default_scenario())

# ══════════════════════════════════════════════════════════════════════════════
#  ORDER MANAGER
# ══════════════════════════════════════════════════════════════════════════════

class OrderManager:
    def __init__(self, pts=150): self.pending=0; self.completed=0; self.score=0; self.pts=pts
    def spawn(self):
        if self.pending < 5: self.pending += 1; return True
        return False
    def complete(self):
        if self.pending > 0: self.pending -= 1
        self.completed += 1; self.score += self.pts
    def st(self): return {"pending":self.pending,"completed":self.completed,"score":self.score}
    def reset(self): self.pending=self.completed=self.score=0

# ══════════════════════════════════════════════════════════════════════════════
#  APP
# ══════════════════════════════════════════════════════════════════════════════

app = Ursina(title="RoboKitchen — MLH 2026 | RoBorregos", size=(1280,720), development_mode=True)
window.color = color.dark_gray
window.fps_counter.enabled = False
window.exit_button.visible = False

# ── Escena ───────────────────────────────────────────────────────────────────
floor_tex = gen_grid_tex((35,38,48,255), (220,220,235,200))
wall_tex  = gen_grid_tex((130,155,185,255), (210,220,235,200))

Entity(model="quad", scale=(ARENA_HALF*2, ARENA_HALF*2),
       position=(0,FLOOR_Y,0), rotation=(-90,0,0), texture=floor_tex, double_sided=True)

walls = []
for f in [
    {"pos":(0,0,ARENA_HALF),"rot":(0,180,0),"eje":"z","dir":1},
    {"pos":(0,0,-ARENA_HALF),"rot":(0,0,0),"eje":"z","dir":-1},
    {"pos":(ARENA_HALF,0,0),"rot":(0,90,0),"eje":"x","dir":1},
    {"pos":(-ARENA_HALF,0,0),"rot":(0,-90,0),"eje":"x","dir":-1},
]:
    w = Entity(model="quad", scale=(ARENA_HALF*2, WALL_H),
               position=f["pos"], rotation=f["rot"], texture=wall_tex, double_sided=True)
    w.eje, w.dir = f["eje"], f["dir"]; walls.append(w)

Entity(model="cube", color=color.gray,
       position=(0,0,-1.25), scale=(0.1,WALL_H,WALL_H/2),
       unlit=True, edge_color=color.black, edge_width=2)

# ── Estaciones (guardar referencias para Scenario) ───────────────────────────
stations_vis = {}
stations_text = {}
stations_body = {}

def _make_station(name, pos, clr):
    e = Entity(model="cube", color=clr, position=pos, scale=1,
               unlit=True, edge_color=color.black, edge_width=3)
    t = Text(text=name.upper(), position=(pos[0], pos[1]+0.9, pos[2]),
             origin=(0,0), scale=0.8, color=color.white, billboard=True)
    b = pw.box((0.5, 0.5, 0.5), 0, (pos[0], pos[1], pos[2]))
    stations_vis[name] = e
    stations_text[name] = t
    stations_body[name] = b
    return e, t, b

_make_station("almacen_tomate",  STATIONS["almacen_tomate"],  color.red)
_make_station("almacen_lechuga", STATIONS["almacen_lechuga"], color.green)
_make_station("corte",           STATIONS["corte"],           color.yellow)
_make_station("ensamblaje",      STATIONS["ensamblaje"],      color.brown)
_make_station("platos",          STATIONS["platos"],          color.white)
_make_station("entrega",         STATIONS["entrega"],         color.azure)

Text(text=("<red>Rojo:<default> Tomate\n<green>Verde:<default> Lechuga\n"
           "<yellow>Amarillo:<default> Corte\n<brown>Cafe:<default> Ensamble\n"
           "Blanco: Platos\n<azure>Azul:<default> Entrega\nGris: Pared"),
     position=(0.55,0.42), origin=(-0.5,0.5), scale=0.85, background=True)

AmbientLight(color=color.rgba(190,190,210,255))
dl = DirectionalLight(); dl.look_at(Vec3(0.5,-1,-0.5))

pivot = Entity()
camera.parent = pivot
camera.position = (0,0,-18)
pivot.rotation_x, pivot.rotation_y = 35, 45

# ── Física ───────────────────────────────────────────────────────────────────
pw = PhysicsWorld()
hw = WALL_H/2

# Piso físico
pw.box((ARENA_HALF + 0.5, 0.1, ARENA_HALF + 0.5), 0, (0, FLOOR_Y - 0.1, 0))

# Paredes perimetrales
for wx, wz in [(ARENA_HALF + 0.15, 0), (-(ARENA_HALF + 0.15), 0),
               (0, ARENA_HALF + 0.15), (0, -(ARENA_HALF + 0.15))]:
    hx, hz = (0.15, ARENA_HALF + 0.15) if wx == 0 else (ARENA_HALF + 0.15, 0.15)
    pw.box((hx, hw, hz), 0, (wx, 0, wz))

# Pared interna
pw.box((0.05, hw, 1.25), 0, (0, 0, -1.25))

# ── Robots ───────────────────────────────────────────────────────────────────
roles = ["recolector","cortador","ensamblador","repartidor"]
robots = [Robot(pw, i, r, ROBOT_STARTS[i]) for i, r in enumerate(roles)]

# ── Ingredients ──────────────────────────────────────────────────────────────
ingredients = [
    Ingredient(pw, "lechuga", STATIONS["almacen_lechuga"]),
    Ingredient(pw, "tomate",  STATIONS["almacen_tomate"]),
]

# ── Plates ───────────────────────────────────────────────────────────────────
class Plate:
    def __init__(self, pw, spawn):
        self.pw = pw; self.spawn = spawn; self.held = None; self.food = False
        self.body = pw.box((0.3, 0.05, 0.3), 0, (spawn.x, spawn.y + 0.15, spawn.z))
        self.vis = Entity(model="cube", color=color.rgb(240, 240, 250), scale=(0.6, 0.1, 0.6),
                          position=spawn + Vec3(0, 0.3, 0),
                          unlit=True, edge_color=color.black, edge_width=2)

    def pos(self):
        if self.held: return self.held.pos() + Vec3(0, 0.5, 0)
        return self.spawn + Vec3(0, 0.3, 0)

    def sync(self):
        if not self.held:
            self.pw.reset(self.body, (self.spawn.x, self.spawn.y + 0.15, self.spawn.z))

    def pickup(self, robot):
        self.held = robot; robot.carrying_plate = self
        self.vis.parent = robot.vis; self.vis.position = Vec3(0, 1.0, 0)

    def drop(self, wpos):
        if self.held:
            self.vis.parent = None; self.held.carrying_plate = None; self.held = None
            self.pw.reset(self.body, (wpos.x, wpos.y + 0.15, wpos.z))
            self.vis.position = Vec3(wpos.x, wpos.y + 0.3, wpos.z)

    def reset(self):
        if self.held:
            self.vis.parent = None; self.held.carrying_plate = None; self.held = None
        self.food = False
        self.pw.reset(self.body, (self.spawn.x, self.spawn.y + 0.15, self.spawn.z))
        self.vis.position = self.spawn + Vec3(0, 0.3, 0)

plates = [Plate(pw, STATIONS["platos"])]

# ── Scenario ─────────────────────────────────────────────────────────────────
scenario = Scenario(pw)
scenario.set_refs(stations_vis, stations_text, stations_body)

# ── Gemini ───────────────────────────────────────────────────────────────────
gemini = GeminiAgent()

# ── Órdenes ──────────────────────────────────────────────────────────────────
orders = OrderManager()
orders.spawn()

# ── HUD robots ───────────────────────────────────────────────────────────────
robot_ui = []
for i, rl in enumerate(roles):
    t = Text(text=rl.upper(), position=(-0.75, 0.45-i*0.055),
             origin=(-0.5,0.5), scale=0.8); robot_ui.append(t)
score_txt = Text(text="SCORE: 0 | PEDIDOS: 1", position=(-0.75, 0.22),
                 origin=(-0.5,0.5), scale=0.9, color=color.yellow)
gemini_txt = Text(text="Gemini: ON" if gemini._ok else "Gemini: OFF",
                  position=(-0.75, 0.16), origin=(-0.5,0.5), scale=0.8,
                  color=color.green if gemini._ok else color.red)

# ══════════════════════════════════════════════════════════════════════════════
#  HELPERS (game loop)
# ══════════════════════════════════════════════════════════════════════════════

def _clamp_speed(r):
    v = pw.vel(r.body); s = (v[0]**2+v[2]**2)**0.5
    mx = ROBOT_SPEEDS[r.role]
    if s > mx:
        f = mx/s
        p.resetBaseVelocity(r.body, linearVelocity=(v[0]*f,v[1],v[2]*f),
                            angularVelocity=(0,0,0), physicsClientId=pw.cli)

def _update_stations():
    for r in robots:
        pp = r.pos()
        found = False
        for name, pos in STATIONS.items():
            if abs(pp.x - pos.x) < 0.7 and abs(pp.z - pos.z) < 0.7:
                r.station = name
                found = True
                break
        if not found:
            if pp.x > 0 and pp.z < -1.0:
                r.station = "almacen"
            else:
                r.station = "pasillo"

def _navigate(r, tgt):
    pp = r.pos(); dx = tgt.x-pp.x; dz = tgt.z-pp.z
    d = (dx*dx+dz*dz)**0.5
    if d < 0.7: r.stop(); return
    f = ROBOT_FORCES[r.role]
    pw.force(r.body, (dx/d*f, 0, dz/d*f)); _clamp_speed(r)

def _try_pickup(r):
    if r.carrying: return
    for ing in ingredients:
        if ing.held or ing.state != "crudo": continue
        if (r.pos()-ing.pos()).length() < 1.5:
            r.pickup(ing); return

def _try_pickup_plate(r):
    for p in plates:
        if p.held or p.food: continue
        if (r.pos()-p.pos()).length() < 1.5:
            p.pickup(r); return

def _check_obstacles(r):
    """Pathfinding evasión simple: detecta colisiones cercanas y evade"""
    pp = r.pos()
    # Raycast simple: proyectar puntos adelante
    v = pw.vel(r.body)
    speed = (v[0]**2 + v[2]**2)**0.5
    if speed < 0.5: return
    # Normalizar velocidad
    vx, vz = v[0]/speed, v[2]/speed
    # Proyectar 3 posiciones adelante (centro, izq, der)
    for dist in [0.8, 1.2]:
        cx = pp.x + vx * dist
        cz = pp.z + vz * dist
        # Verificar si hay obstáculo cerca en PyBullet
        # Hacemos un overlap test con aabb simple
        for obs_body in scenario.obstacle_body:
            op = pw.pos(obs_body)
            if abs(op[0] - cx) < 0.4 and abs(op[2] - cz) < 0.4:
                r.avoid()
                return
        # También verificar otras estaciones (que ya tienen cuerpos)
        for name, body in stations_body.items():
            if name in ("platos","entrega"): continue
            op = pw.pos(body)
            if abs(op[0] - cx) < 0.5 and abs(op[2] - cz) < 0.5:
                r.avoid()
                return

def _pipeline():
    ensamblador = robots[2]
    repartidor = robots[3]

    for r in robots:
        _check_obstacles(r)
        if r.role == "recolector" and not r.carrying:
            crudos = [i for i in ingredients if i.state=="crudo" and not i.held]
            if crudos and orders.pending > 0:
                _navigate(r, crudos[0].spawn)
                if r.station in ("almacen_tomate","almacen_lechuga","almacen"): _try_pickup(r)
        if r.role == "cortador" and not r.carrying:
            _navigate(r, STATIONS["corte"])
            if r.station == "corte":
                for ing in ingredients:
                    if not ing.held and ing.state == "crudo":
                        if (r.pos()-ing.pos()).length() < 1.5: r.pickup(ing)

    # Ensamblador
    if ensamblador.carrying_plate and ensamblador.carrying_plate.food:
        if ensamblador.station == "ensamblaje":
            ensamblador.carrying_plate.drop(ensamblador.pos())
        else:
            _navigate(ensamblador, STATIONS["ensamblaje"])
    elif not ensamblador.carrying_plate:
        free_plates = [p for p in plates if not p.held and not p.food]
        if free_plates:
            _navigate(ensamblador, STATIONS["platos"])
            if ensamblador.station == "platos": _try_pickup_plate(ensamblador)
    elif ensamblador.station == "ensamblaje" and ensamblador.carrying_plate and not ensamblador.carrying_plate.food:
        cort = [i for i in ingredients if i.state=="cortado" and not i.held]
        lech = [i for i in cort if i.itype=="lechuga"]
        tom = [i for i in cort if i.itype=="tomate"]
        if lech and tom:
            ensamblador.carrying_plate.food = True
            ensamblador.carrying_plate.vis.color = color.rgb(255, 210, 60)
            lech[0].reset(); tom[0].reset()
            ensamblador.carrying_plate.drop(ensamblador.pos())

    # Repartidor
    assembled = [p for p in plates if p.food and not p.held]
    if assembled and not repartidor.carrying_plate:
        _navigate(repartidor, assembled[0].pos())
        if (repartidor.pos() - assembled[0].pos()).length() < 1.5:
            assembled[0].pickup(repartidor)
    elif repartidor.carrying_plate:
        _navigate(repartidor, STATIONS["entrega"])

# ══════════════════════════════════════════════════════════════════════════════
#  UPDATE (Ursina game loop)
# ══════════════════════════════════════════════════════════════════════════════

def update():
    global gemini
    dt = utime.dt
    if dt <= 0 or dt > 0.1: return

    # ── Cámara orbital ──────────────────────────────────────────────────
    sp = 70 * dt
    pivot.rotation_y += (held_keys["d"]-held_keys["a"]+
                          held_keys["right arrow"]-held_keys["left arrow"]) * sp
    pivot.rotation_x += (held_keys["w"]-held_keys["s"]+
                          held_keys["up arrow"]-held_keys["down arrow"]) * sp
    pivot.rotation_x = clamp(pivot.rotation_x, 10, 85)

    # ── Paredes invisibles ──────────────────────────────────────────────
    cp = camera.world_position
    for w in walls:
        if w.eje == "x":
            w.visible = not ((w.dir==1 and cp.x>w.position.x) or
                             (w.dir==-1 and cp.x<w.position.x))
        elif w.eje == "z":
            w.visible = not ((w.dir==1 and cp.z>w.position.z) or
                             (w.dir==-1 and cp.z<w.position.z))

    # ── Input ───────────────────────────────────────────────────────────
    if held_keys["r"]:
        # Presionar R = nuevo escenario (Gemini o default)
        print("[R] Generando nuevo escenario...")
        cfg = gemini.generate_scenario()
        scenario.apply(cfg)
        orders.reset(); orders.spawn()
        for r in robots: r.reset()
        for i in ingredients: i.reset()
        for p in plates: p.reset()
        gemini_txt.text = f"Gemini: {'ON' if gemini._ok else 'OFF'}"
        gemini_txt.color = color.green if gemini._ok else color.red

    # ── Física ──────────────────────────────────────────────────────────
    for _ in range(min(max(1, int(dt*240)), 8)): pw.step()

    # ── Procesar ingredientes ───────────────────────────────────────────
    for ing in ingredients:
        if ing.proc and ing.state=="crudo" and not ing.held:
            ing.proc_t -= dt
            if ing.proc_t <= 0: ing.set_state("cortado"); ing.proc = False

    # ── Pipeline automático ─────────────────────────────────────────────
    _pipeline()

    # ── Detectar entregas ───────────────────────────────────────────────
    entrega = STATIONS["entrega"]
    for r in robots:
        if r.carrying_plate and r.carrying_plate.food:
            if (r.pos()-entrega).length() < 1.5:
                r.carrying_plate.drop(r.pos())
                orders.complete()
                print(f"[Order] +{orders.pts}pts | total={orders.completed}")
                for i in ingredients: i.reset()
                for p in plates: p.reset()
                orders.spawn()

    # ── Sincronizar visuales ────────────────────────────────────────────
    _update_stations()
    for r in robots: r.sync()
    for i in ingredients: i.sync()
    for p in plates: p.sync()

    # ── UI ──────────────────────────────────────────────────────────────
    for i, r in enumerate(robots):
        c = "+" if (r.carrying or r.carrying_plate) else "-"
        a = r.action[:6]; s = r.station[:6]
        robot_ui[i].text = f"R{i} {r.role[:3].upper()} | {s} | {a} [{c}]"
    score_txt.text = f"SCORE: {orders.score} | PEDIDOS: {orders.pending}"

# ══════════════════════════════════════════════════════════════════════════════

app.run()
