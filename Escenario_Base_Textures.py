from ursina import *

app = Ursina()

# --- CONFIGURACIÓN ---
size = 5
half = size / 2

# Nombres de los archivos (sin extensión) que Ursina buscará en la carpeta 'textures'
wall_tex = 'textura_pared' 
floor_tex = 'textura_piso'

# --- CONSTRUCCIÓN DEL ENTORNO (Paredes externas) ---
faces_data = [
    {'pos': (0, 0, half),  'rot': (0, 180, 0), 'eje': 'z', 'dir': 1,  'tex': wall_tex},
    {'pos': (0, 0, -half), 'rot': (0, 0, 0),   'eje': 'z', 'dir': -1, 'tex': wall_tex},
    {'pos': (half, 0, 0),  'rot': (0, 90, 0),  'eje': 'x', 'dir': 1,  'tex': wall_tex},
    {'pos': (-half, 0, 0), 'rot': (0, -90, 0), 'eje': 'x', 'dir': -1, 'tex': wall_tex},
    {'pos': (0, -half, 0), 'rot': (-90, 0, 0), 'eje': 'y', 'dir': -1, 'tex': floor_tex},
]

walls = []
for face in faces_data:
    w = Entity(
        model='quad', 
        scale=(size, size), 
        position=face['pos'],
        rotation=face['rot'], 
        texture=face['tex'], # <-- Se asigna la imagen que tienes en tu carpeta
        double_sided=True
    )
    
    # IMPORTANTE: Esto hace que la textura se repita (tiling) como azulejos.
    # Si la imagen se ve muy chica o muy grande, puedes cambiar (size, size) por (2, 2) o (1, 1).
    w.texture_scale = (size, size) 
    
    w.eje, w.dir = face['eje'], face['dir']
    walls.append(w)

# --- FUNCIÓN DE CUBOS CON BORDE (Estilo Dibujo) ---
def Station(pos, col):
    return Entity(
        model='cube',
        color=col,
        position=pos,
        scale=1,
        unlit=True,             # Color plano sin sombras
        edge_color=color.black, # Borde negro estilo dibujo
        edge_width=3            
    )

# --- INSTANCIACIÓN DE OBJETOS ---
Station((1, -half + 0.5, -2), color.red)      # Tomate
Station((2, -half + 0.5, -2), color.green)    # Lechuga
Station((-2, -half + 0.5, -2), color.yellow)  # Cutting Station
Station((-2, -half + 0.5, -1), color.brown)   # Ensamblaje
Station((-2, -half + 0.5, 0), color.white)    # Platos
Station((2, -half + 0.5, 0), color.azure)     # Deliver 1
Station((2, -half + 0.5, 1), color.azure)     # Deliver 2

# Pared interna (Desde centro hasta el fondo)
internal_wall = Entity(
    model='cube',
    color=color.gray,
    position=(0, -half + 2.5, -1.25), 
    scale=(0.1, 5, 2.5),
    unlit=True,
    edge_color=color.black,
    edge_width=2
)

# --- INTERFAZ DE USUARIO ---
window.color = color.dark_gray # Fondo gris oscuro conservado

Text(text="CONTROLES: WASD / Flechas", position=(-0.75, 0.45), origin=(-0.5, 0.5), scale=1.1, background=True)

leyenda_texto = (
    "<red>Rojo:<default> Tomate\n"
    "<green>Verde:<default> Lechuga\n"
    "<yellow>Amarillo:<default> Corte\n"
    "<brown>Cafe:<default> Ensamblaje\n"
    "Blanco: Platos\n"
    "<azure>Azul:<default> Entrega\n"
    "Gris: Pared"
)

Text(
    text=leyenda_texto,
    position=(0.55, 0.4), 
    origin=(-0.5, 0.5),
    scale=0.9,
    background=True
)

# --- SISTEMA DE CÁMARA ORBITAL ---
pivot = Entity()
camera.parent = pivot
camera.position = (0, 0, -18) 
pivot.rotation_x, pivot.rotation_y = 35, 45

def update():
    # Rotación de cámara
    rot_speed = 100 * time.dt
    pivot.rotation_y += (held_keys['d'] - held_keys['a'] + held_keys['right arrow'] - held_keys['left arrow']) * rot_speed
    pivot.rotation_x += (held_keys['w'] - held_keys['s'] + held_keys['up arrow'] - held_keys['down arrow']) * rot_speed
    pivot.rotation_x = clamp(pivot.rotation_x, 10, 85)

    # Ocultar paredes que obstruyen la vista
    cam_pos = camera.world_position
    for w in walls:
        if w.eje == 'x':
            w.enabled = not ((w.dir == 1 and cam_pos.x > w.position.x) or (w.dir == -1 and cam_pos.x < w.position.x))
        elif w.eje == 'z':
            w.enabled = not ((w.dir == 1 and cam_pos.z > w.position.z) or (w.dir == -1 and cam_pos.z < w.position.z))

app.run()