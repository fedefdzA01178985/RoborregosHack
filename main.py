"""
RoboKitchen — MLH Hackathon 2026 | RoBorregos | Tec de Monterrey
Overcooked 3D: 4 robots pipeline preparando ensaladas.
Ursina (visual) + PyBullet (fisica) + Gemini (IA estrategica).
"""

import threading, time, json, sys

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
    Vec3(2.0,  ROBOT_Y, 2.2),   # recolector  — lejos almacen
    Vec3(0.7,  ROBOT_Y, 2.2),   # cortador    — lejos corte
    Vec3(-0.7, ROBOT_Y, 2.2),   # ensamblador — lejos ensamblaje/platos
    Vec3(-2.0, ROBOT_Y, 2.2),   # repartidor  — lejos entrega
]
STATIONS = {
    "almacen_tomate":  Vec3(1.0, OBJ_Y, -2.0),
    "almacen_lechuga": Vec3(2.0, OBJ_Y, -2.0),
    "corte":           Vec3(-2.0, OBJ_Y, -2.0),
    "ensamblaje":      Vec3(-2.0, OBJ_Y, -1.0),
    "platos":          Vec3(-2.0, OBJ_Y, 0.0),
    "entrega":         Vec3(2.0, OBJ_Y, 1.0),
}
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
        self.body = pw.box((0.4, 0.35, 0.5), 3.0, (start.x, start.y, start.z),
                           ldamp=0.5, adamp=0.9)
        self.vis = Entity(model="cube", color=ROBOT_COLORS[role],
                          scale=(0.8, 0.7, 1.0), position=start,
                          unlit=True, edge_color=color.black, edge_width=2)
        self.dot = Entity(parent=self.vis, model="quad", color=color.yellow,
                          scale=(0.3, 0.3), position=Vec3(0, 0.9, 0),
                          billboard=True, enabled=False)

    def pos(self): p = self.pw.pos(self.body); return Vec3(p[0], p[1], p[2])

    def sync(self):
        p = self.pw.pos(self.body); self.vis.position = Vec3(p[0], p[1], p[2])
        self.dot.enabled = self.carrying is not None or self.carrying_plate is not None

    def move_to(self, target):
        pp = self.pos(); dx = target.x - pp.x; dz = target.z - pp.z
        d = (dx*dx + dz*dz)**0.5
        if d < 0.4: self.stop(); return
        f = ROBOT_FORCES[self.role]
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
#  GEMINI AGENT
# ══════════════════════════════════════════════════════════════════════════════

class GeminiAgent:
    SYS = (
        "Eres el chef de RoboKitchen: 4 robots hacen ensaladas en cadena.\n"
        "ROBOTS: 0=RECOLECTOR(rapido),1=CORTADOR(medio),2=ENSAMBLADOR(medio),3=REPARTIDOR(rapido).\n"
        "PIPELINE: ALMACEN(x>0,z=-2)->RECOGER->CORTE(-2,-2)->CORTAR 2s->ENSAMBLAJE(-2,-1)->ARMAR->PLATO->ENTREGA(2,1).\n"
        "PLATOS(-2,0) = donde ENSAMBLADOR recoge plato vacio para llevar a ENSAMBLAJE.\n"
        "PARED en x=0 de z=-2.5 a z=0. Rodear por z>0.\n"
        "RESPONDE SOLO JSON array: [{\"robot_id\":0,\"action\":\"pickup\",\"target\":[x,z]}, ...]\n"
        "Acciones: pickup, deliver, process, assemble, pickup_plate, goto, wait, idle"
    )
    PROJECT = "neural-pattern-495817-k4"
    LOCATION = "us-central1"
    MODEL = "gemini-2.0-flash"

    def __init__(self):
        import os
        from google import genai
        from google.genai import types
        self._genai = genai
        self._types = types
        os.environ.setdefault("GOOGLE_GENAI_USE_VERTEXAI", "true")
        os.environ.setdefault("GOOGLE_CLOUD_PROJECT", self.PROJECT)
        os.environ.setdefault("GOOGLE_CLOUD_LOCATION", self.LOCATION)
        self.calls = 0; self._err_shown = False; self._ok = False
        try:
            self.client = genai.Client(vertexai=True, project=self.PROJECT, location=self.LOCATION)
            test = self.client.models.generate_content(
                model=self.MODEL,
                contents="Responde solo: OK",
                config=types.GenerateContentConfig(temperature=0, max_output_tokens=10))
            if "OK" in (test.text or "").upper():
                self._ok = True; print("[Gemini] Vertex AI OK — listo para usar")
            else:
                print("[Gemini] Vertex respondio pero sin OK — fallback manual activo")
        except Exception as e:
            print(f"[Gemini] Vertex NO disponible ({type(e).__name__}) — fallback manual activo")

    def decide(self, state):
        self.calls += 1
        prompt = f"{self.SYS}\n=== #{self.calls} ===\n{json.dumps(state,indent=2)}\n\nJSON array:"
        try:
            r = self.client.models.generate_content(
                model=self.MODEL,
                contents=prompt,
                config=self._types.GenerateContentConfig(temperature=0.2, max_output_tokens=300))
            return self._parse(r.text)
        except Exception as e:
            if not self._err_shown:
                print(f"[Gemini] Error Vertex ({type(e).__name__}) — usando pipeline manual")
                self._err_shown = True
            return self._fallback(state)

    def _parse(self, t):
        t = (t or "").strip()
        if t.startswith("```"):
            p = t.split("```"); t = p[1] if len(p)>=2 else t
            if t.startswith("json"): t = t[4:]
        t = t.strip()
        try: d = json.loads(t)
        except:
            import re; m = re.search(r"\[.*\]", t, re.DOTALL)
            if m: d = json.loads(m.group())
            else: return []
        return [{"robot_id":int(c.get("robot_id",0)),"action":c.get("action","idle"),
                 "target":c.get("target"), "station":c.get("station",c.get("target"))}
                for c in d if isinstance(c,dict)]

    def _fallback(self, s):
        rs = {r["id"]:r for r in s.get("robots",[])}; pending = s.get("orders",{}).get("pending",0)
        if pending <= 0: return [{"robot_id":i,"action":"idle"} for i in range(4)]
        cmds = []
        r0 = rs.get(0,{}); r1 = rs.get(1,{}); r2 = rs.get(2,{}); r3 = rs.get(3,{})
        ings = s.get("ingredients",[])
        crudos = [i for i in ings if i.get("state")=="crudo" and not i.get("held_by")]
        cortados = [i for i in ings if i.get("state")=="cortado" and not i.get("held_by")]
        # 0 recolector
        if crudos and not r0.get("carrying"):
            cmds.append({"robot_id":0,"action":"pickup","target":crudos[0]["pos"]})
        elif r0.get("carrying"):
            cmds.append({"robot_id":0,"action":"deliver","target":"corte"})
        else: cmds.append({"robot_id":0,"action":"idle"})
        # 1 cortador
        if r1.get("carrying"):
            cmds.append({"robot_id":1,"action":"process"})
        else:
            cmds.append({"robot_id":1,"action":"goto","target":"corte"})
        # 2 ensamblador
        if not r2.get("carrying_plate") and not r2.get("carrying"):
            cmds.append({"robot_id":2,"action":"pickup_plate","target":"platos"})
        elif r2.get("carrying_plate") and cortados:
            cmds.append({"robot_id":2,"action":"assemble","target":"ensamblaje"})
        else: cmds.append({"robot_id":2,"action":"idle"})
        # 3 repartidor
        if not r3.get("carrying_plate") and not r3.get("carrying"):
            cmds.append({"robot_id":3,"action":"goto","target":"ensamblaje"})
        elif r3.get("carrying_plate"):
            cmds.append({"robot_id":3,"action":"deliver","target":"entrega"})
        else: cmds.append({"robot_id":3,"action":"idle"})
        return cmds

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

stations_data = [
    ((1,OBJ_Y,-2),  color.red,    "TOMATE"),
    ((2,OBJ_Y,-2),  color.green,  "LECHUGA"),
    ((-2,OBJ_Y,-2), color.yellow, "CORTE"),
    ((-2,OBJ_Y,-1), color.brown,  "ENSAMBLE"),
    ((-2,OBJ_Y,0),  color.white,  "PLATOS"),
    ((2,OBJ_Y,1),   color.azure,  "ENTREGA"),
]
for pos, clr, lbl in stations_data:
    Entity(model="cube", color=clr, position=pos, scale=1,
           unlit=True, edge_color=color.black, edge_width=3)
    Text(text=lbl, position=(pos[0], pos[1]+0.9, pos[2]),
         origin=(0,0), scale=0.8, color=color.white, billboard=True)

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
hw = WALL_H/2  # 2.5

# Piso físico en y = -2.5 (misma altura que el quad visual)
pw.box((ARENA_HALF + 0.5, 0.1, ARENA_HALF + 0.5), 0, (0, FLOOR_Y - 0.1, 0))

# Paredes perimetrales (física alineada con visual: centro y=0, altura 5)
for wx, wz in [(ARENA_HALF + 0.15, 0), (-(ARENA_HALF + 0.15), 0),
               (0, ARENA_HALF + 0.15), (0, -(ARENA_HALF + 0.15))]:
    hx, hz = (0.15, ARENA_HALF + 0.15) if wx == 0 else (ARENA_HALF + 0.15, 0.15)
    pw.box((hx, hw, hz), 0, (wx, 0, wz))

# Pared interna (física alineada con visual: centro y=0)
pw.box((0.05, hw, 1.25), 0, (0, 0, -1.25))

# Colisiones estaciones (para que robots no atraviesen los cubos)
for pos, _, _ in stations_data:
    pw.box((0.5, 0.5, 0.5), 0, (pos[0], pos[1], pos[2]))

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

# ── Órdenes + Gemini ─────────────────────────────────────────────────────────
orders = OrderManager()
orders.spawn()

gemini_on = True
try: gemini = GeminiAgent()
except Exception as e: print(f"[Gemini] Off: {e}"); gemini_on = False; gemini = None

_glock = threading.Lock()
_cmds = []
_last_g = 0.0

# ── HUD robots ───────────────────────────────────────────────────────────────
robot_ui = []
for i, rl in enumerate(roles):
    t = Text(text=rl.upper(), position=(-0.75, 0.45-i*0.055),
             origin=(-0.5,0.5), scale=0.8); robot_ui.append(t)
score_txt = Text(text="SCORE: 0 | PEDIDOS: 1", position=(-0.75, 0.22),
                 origin=(-0.5,0.5), scale=0.9, color=color.yellow)
gemini_txt = Text(text="Gemini: ON", position=(-0.75, 0.16),
                  origin=(-0.5,0.5), scale=0.8, color=color.green)

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

def _resolve_target(tgt, stn):
    if isinstance(tgt, (list,tuple)) and len(tgt)>=2:
        return Vec3(tgt[0], OBJ_Y, tgt[-1])
    if isinstance(tgt, str):
        st = STATIONS.get(tgt) or STATIONS.get(stn)
        return st
    return None

def _update_stations():
    for r in robots:
        pp = r.pos()
        if abs(pp.x-(-2.0))<0.7 and abs(pp.z-(-2.0))<0.7: r.station = "corte"
        elif abs(pp.x-(-2.0))<0.7 and abs(pp.z-(-1.0))<0.7: r.station = "ensamblaje"
        elif abs(pp.x-2.0)<0.7 and abs(pp.z-1.0)<0.7: r.station = "entrega"
        elif abs(pp.x-(-2.0))<0.7 and abs(pp.z-0.0)<0.7: r.station = "platos"
        elif pp.x>0 and pp.z<-1.0: r.station = "almacen"
        else: r.station = "pasillo"

def _state():
    rs = []
    for r in robots:
        pp = r.pos()
        rs.append({"id":r.rid,"role":r.role,"pos":[round(pp.x,2),round(pp.z,2)],
                   "carrying":r.carrying.itype if r.carrying else None,
                   "station":r.station,"action":r.action})
    igs = []
    for i in ingredients:
        pp = i.pos()
        igs.append({"type":i.itype,"state":i.state,"pos":[round(pp.x,2),round(pp.z,2)],
                    "held_by":i.held.rid if i.held else None})
    return {"orders":orders.st(),"robots":rs,"ingredients":igs,"score":orders.score}

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

def _apply_cmds():
    global _cmds
    with _glock: cmds = _cmds.copy(); _cmds.clear()
    now = time.time()
    for c in cmds:
        rid = c.get("robot_id",0); act = c.get("action","idle")
        tgt = c.get("target"); stn = c.get("station",tgt)
        if rid >= len(robots): continue
        r = robots[rid]; r.action = act; r._last_cmd_t = now
        if act in ("goto","pickup","deliver"):
            tp = _resolve_target(tgt, stn)
            if tp is None: r.stop(); continue
            _navigate(r, tp)
            if act == "pickup":
                if r.role == "ensamblador" and r.station == "platos":
                    _try_pickup_plate(r)
                elif r.station in ("almacen","corte"):
                    _try_pickup(r)
            elif act == "deliver" and (r.pos()-tp).length() < 1.0:
                if r.carrying: r.drop(r.pos())
        elif act == "process" and r.station == "corte" and r.carrying:
            if not r.carrying.proc:
                r.carrying.proc = True; r.carrying.proc_t = 2.0
        elif act == "assemble" and r.station == "ensamblaje":
            cort = [i for i in ingredients if i.state=="cortado" and not i.held]
            lech = [i for i in cort if i.itype=="lechuga"]
            tom = [i for i in cort if i.itype=="tomate"]
            if lech and tom and r.carrying_plate and not r.carrying_plate.food:
                r.carrying_plate.food = True
                r.carrying_plate.vis.color = color.rgb(255, 210, 60)
                lech[0].reset(); tom[0].reset()
                r.carrying_plate.drop(r.pos())
        elif act == "pickup_plate" and r.station in ("platos","ensamblaje"):
            _try_pickup_plate(r)
        else: r.stop()

def _try_pickup_plate(r):
    for p in plates:
        if p.held or p.food: continue
        if (r.pos()-p.pos()).length() < 1.5:
            p.pickup(r); return

def _pipeline():
    now = time.time()
    ensamblador = robots[2]
    repartidor = robots[3]

    for r in robots:
        # skip if Gemini gave a command recently (<1s)
        if getattr(r, "_last_cmd_t", 0) and now - r._last_cmd_t < 1.0:
            continue
        if r.role == "recolector" and not r.carrying:
            crudos = [i for i in ingredients if i.state=="crudo" and not i.held]
            if crudos and orders.pending > 0:
                _navigate(r, crudos[0].spawn)
                if r.station == "almacen": _try_pickup(r)
        if r.role == "cortador" and not r.carrying:
            _navigate(r, STATIONS["corte"])
            if r.station == "corte":
                for ing in ingredients:
                    if not ing.held and ing.state == "crudo":
                        if (r.pos()-ing.pos()).length() < 1.5: r.pickup(ing)

    if not (getattr(ensamblador, "_last_cmd_t", 0) and now - ensamblador._last_cmd_t < 1.0):
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
                # try assemble if ingredients ready
                cort = [i for i in ingredients if i.state=="cortado" and not i.held]
                lech = [i for i in cort if i.itype=="lechuga"]
                tom = [i for i in cort if i.itype=="tomate"]
                if lech and tom:
                    ensamblador.carrying_plate.food = True
                    ensamblador.carrying_plate.vis.color = color.rgb(255, 210, 60)
                    lech[0].reset(); tom[0].reset()
                    ensamblador.carrying_plate.drop(ensamblador.pos())

    if not (getattr(repartidor, "_last_cmd_t", 0) and now - repartidor._last_cmd_t < 1.0):
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
    global gemini_on, _last_g, _cmds
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
        orders.reset(); orders.spawn()
        for r in robots: r.reset()
        for i in ingredients: i.reset()
        for p in plates: p.reset()
    if held_keys["g"]:
        gemini_on = not gemini_on
        print(f"[Gemini] {'ON' if gemini_on else 'OFF'}")

    # ── Física ──────────────────────────────────────────────────────────
    with _glock:
        for _ in range(min(max(1, int(dt*240)), 8)): pw.step()

    # ── Gemini (cada 3s) ────────────────────────────────────────────────
    if gemini_on and gemini:
        now = time.time()
        if now - _last_g >= 3.0:
            _last_g = now
            st = _state()
            def _call(): 
                try:
                    cmds = gemini.decide(st)
                    if cmds:
                        with _glock: _cmds.extend(cmds)
                except Exception as e: print(f"[Gemini] thread: {e}")
            threading.Thread(target=_call, daemon=True).start()

    # ── Ejecutar comandos ───────────────────────────────────────────────
    _apply_cmds()

    # ── Procesar ingredientes ───────────────────────────────────────────
    for ing in ingredients:
        if ing.proc and ing.state=="crudo" and not ing.held:
            ing.proc_t -= dt
            if ing.proc_t <= 0: ing.set_state("cortado"); ing.proc = False

    # ── Pipeline (fallback sin Gemini) ──────────────────────────────────
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
    gc = color.green if gemini_on else color.red
    gemini_txt.text = f"Gemini: {'ON' if gemini_on else 'OFF'}"
    gemini_txt.color = gc

# ══════════════════════════════════════════════════════════════════════════════

app.run()
