# Persona 3 — UI + Docs + Video

## Tus archivos

```
ui/
├── __init__.py        (ya creado, no tocar)
├── hud.py             (ya creado)
└── scoreboard.py      (ya creado)

Raiz del proyecto:
├── README.md          (TU lo creas)
├── .env.example       (ya creado)
├── .gitignore         (ya creado)
└── requirements.txt   (ya creado, verifica)
```

## Instalacion rapida (3 pasos)

```powershell
# 1. Clonar y entrar
git clone https://github.com/fedefdzA01178985/RoborregosHack.git
cd RoborregosHack

# 2. Entorno virtual + dependencias
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt

# 3. Gemini API Key
copy .env.example .env
# Editar .env con notepad o VS Code:
# GEMINI_API_KEY=tu_key_de_https://aistudio.google.com/apikey
```

## Que necesitas hacer

### ui/hud.py
- **Estado:** COMPLETO (puedes mejorar visuales)
- HUD que muestra: FPS, score, pedidos pendientes/completados, estado de cada robot, Gemini ON/OFF.
- Usa `Text()` de Ursina.
- **TU TAREA:** Mejorar colores, posiciones, fuentes. Hacer que se vea profesional.

### ui/scoreboard.py
- **Estado:** COMPLETO (puedes mejorar visuales)
- Marcador central: "RoboKitchen", score, contador de ensaladas, pantalla Game Over.
- **TU TAREA:** Ajustar estilos. Agregar animacion al completar orden.

### README.md (TU LO CREAS COMPLETO)
Debe incluir:
- Titulo: "RoboKitchen — MLH Hackathon 2026"
- Descripcion: que es, como funciona
- Stack: Ursina + PyBullet + Gemini API
- Arquitectura (diagrama simple en ASCII)
- Instalacion (Windows):
  ```powershell
  git clone ...
  cd RoborregosHack
  python -m venv venv
  venv\Scripts\activate
  pip install -r requirements.txt
  copy .env.example .env
  # editar .env con tu API key
  python main.py
  ```
- Controles: WASD, R, G, ESC
- Estructura de archivos
- Creditos: equipo, roles

### Video demo (2-3 min)
Graba con OBS o similar mostrando:
1. Inicio: arena 3D, 4 robots en posicion, HUD
2. Pipeline: recolectar → cortar → ensamblar → entregar
3. Gemini decidiendo (HUD muestra "Gemini: ON")
4. Game over / score final
