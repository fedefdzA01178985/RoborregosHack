"""
RoboKitchen — MLH Hackathon 2026 | RoBorregos | Tec de Monterrey
FASE 1: Escenario base con cámara orbital, paredes texturizadas,
estaciones Overcooked, paredes invisibles al orbitar.
"""

import sys

try:
    from ursina import (Ursina, Entity, Vec3, color, time as utime,
                        held_keys, Text, camera, window, clamp,
                        AmbientLight, DirectionalLight, Texture)
except ImportError:
    print("[ERROR] Ursina no instalado. pip install ursina")
    sys.exit(1)

try:
    from PIL import Image, ImageDraw
except ImportError:
    print("[ERROR] Pillow no instalado. pip install pillow")
    sys.exit(1)

# ═══════════════════════════════════════════════════════════════════════
#  TEXTURAS
# ═══════════════════════════════════════════════════════════════════════

def gen_grid_tex(bg_rgba, line_rgba, grid_size=5, res=1024):
    img = Image.new("RGBA", (res, res), bg_rgba)
    draw = ImageDraw.Draw(img)
    lw = 6
    for i in range(grid_size + 1):
        pos = int((i / grid_size) * (res - lw / 2))
        draw.line((0, pos, res, pos), fill=line_rgba, width=lw)
        draw.line((pos, 0, pos, res), fill=line_rgba, width=lw)
    return Texture(img)


# ═══════════════════════════════════════════════════════════════════════
#  APP
# ═══════════════════════════════════════════════════════════════════════

app = Ursina(title="RoboKitchen — MLH 2026 | RoBorregos",
             size=(1280, 720), development_mode=True)
window.color = color.dark_gray
window.fps_counter.enabled = False
window.exit_button.visible = False

# ── Texturas ──────────────────────────────────────────────────────────
floor_tex = gen_grid_tex((35, 38, 48, 255), (220, 220, 235, 200))
wall_tex  = gen_grid_tex((130, 155, 185, 255), (210, 220, 235, 200))

# ── Suelo ─────────────────────────────────────────────────────────────
ARENA_HALF = 2.5
WALL_H = 5.0
FLOOR_Y = -2.5

Entity(model="quad", scale=(ARENA_HALF * 2, ARENA_HALF * 2),
       position=(0, FLOOR_Y, 0), rotation=(-90, 0, 0),
       texture=floor_tex, double_sided=True)

# ── Paredes exteriores (quads) ────────────────────────────────────────
faces = [
    {"pos": (0, 0, ARENA_HALF),  "rot": (0, 180, 0),  "eje": "z", "dir": 1},
    {"pos": (0, 0, -ARENA_HALF), "rot": (0, 0, 0),    "eje": "z", "dir": -1},
    {"pos": (ARENA_HALF, 0, 0),  "rot": (0, 90, 0),   "eje": "x", "dir": 1},
    {"pos": (-ARENA_HALF, 0, 0), "rot": (0, -90, 0),  "eje": "x", "dir": -1},
]
walls = []
for f in faces:
    w = Entity(model="quad", scale=(ARENA_HALF * 2, WALL_H),
               position=f["pos"], rotation=f["rot"], texture=wall_tex,
               double_sided=True)
    w.eje, w.dir = f["eje"], f["dir"]
    walls.append(w)

# ── Pared interna (divide la cocina) ──────────────────────────────────
Entity(model="cube", color=color.gray,
       position=(0, 0, -1.25), scale=(0.1, WALL_H, WALL_H / 2),
       unlit=True, edge_color=color.black, edge_width=2)

# ── Estaciones (Overcooked) ───────────────────────────────────────────
OBJ_Y = FLOOR_Y + 0.75

def station(pos, clr, label):
    Entity(model="cube", color=clr, position=(pos[0], OBJ_Y, pos[2]),
           scale=1, unlit=True, edge_color=color.black, edge_width=3)
    Text(text=label, position=(pos[0], OBJ_Y + 0.9, pos[2]),
         origin=(0, 0), scale=0.8, color=color.white, billboard=True)

station((1, 0, -2),  color.red,    "TOMATE")
station((2, 0, -2),  color.green,  "LECHUGA")
station((-2, 0, -2), color.yellow, "CORTE")
station((-2, 0, -1), color.brown,  "ENSAMBLE")
station((2, 0, 1),   color.azure,  "ENTREGA")

# ── UI Leyenda ────────────────────────────────────────────────────────
leyenda = ("<red>Rojo:<default> Tomate\n"
           "<green>Verde:<default> Lechuga\n"
           "<yellow>Amarillo:<default> Corte\n"
           "<brown>Cafe:<default> Ensamble\n"
           "<azure>Azul:<default> Entrega\n"
           "Gris: Pared")
Text(text=leyenda, position=(0.55, 0.42), origin=(-0.5, 0.5),
     scale=0.85, background=True)
Text(text="WASD / Flechas = orbitar camara", position=(-0.75, 0.47),
     origin=(-0.5, 0.5), scale=0.9, background=True)

# ── Iluminación ───────────────────────────────────────────────────────
AmbientLight(color=color.rgba(190, 190, 210, 255))
dl = DirectionalLight()
dl.look_at(Vec3(0.5, -1, -0.5))

# ── Cámara orbital ────────────────────────────────────────────────────
pivot = Entity()
camera.parent = pivot
camera.position = (0, 0, -18)
pivot.rotation_x = 35
pivot.rotation_y = 45


# ═══════════════════════════════════════════════════════════════════════
#  GAME LOOP — Función global (Ursina la llama automáticamente)
# ═══════════════════════════════════════════════════════════════════════

def update():
    # ── Órbita de cámara ──────────────────────────────────────────
    speed = 80 * utime.dt
    pivot.rotation_y += (held_keys["d"] - held_keys["a"] +
                          held_keys["right arrow"] - held_keys["left arrow"]) * speed
    pivot.rotation_x += (held_keys["w"] - held_keys["s"] +
                          held_keys["up arrow"] - held_keys["down arrow"]) * speed
    pivot.rotation_x = clamp(pivot.rotation_x, 10, 85)

    # ── Paredes invisibles al orbitar ──────────────────────────────
    cam_pos = camera.world_position
    for w in walls:
        if w.eje == "x":
            w.visible = not ((w.dir == 1 and cam_pos.x > w.position.x) or
                             (w.dir == -1 and cam_pos.x < w.position.x))
        elif w.eje == "z":
            w.visible = not ((w.dir == 1 and cam_pos.z > w.position.z) or
                             (w.dir == -1 and cam_pos.z < w.position.z))


# ═══════════════════════════════════════════════════════════════════════

app.run()
