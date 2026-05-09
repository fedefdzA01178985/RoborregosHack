# RoboKitchen — Plan de Proyecto

## ¿Qué es?

Una cocina robótica 3D donde 4 robots trabajan en cadena (pipeline) sobre una arena cuadriculada 5×5 de celdas cúbicas de 5×5×5 metros. Preparan ensaladas automáticamente: el **Recolector** recoge ingredientes del almacén, el **Cortador** los rebana (2s), el **Ensamblador** arma el plato, y el **Repartidor** lo entrega. **Gemini** actúa como chef supervisor monitoreando toda la cadena: asigna tareas, detecta cuellos de botella y replanifica si algún robot se atora. El mapa se define en `map_config.json` — cambiar el layout es editar texto.

---

## Arena 3D — 5×5 celdas de 5³m

```
COL→  0       1       2       3       4
  ┌───────┬───────┬───────┬───────┬───────┐
0 │🥬ALMAC│🍅ALMAC│       │📋TABLA│       │
  │ lechuga│ tomate│       │pedidos│       │
  ├───────┼───────┼───────┼───────┼───────┤
1 │       │🔪CORTE│🔪CORTE│       │       │
  │       │  2s   │  2s   │       │       │
  ├───────┼───────┼───────┼───────┼───────┤
2 │       │       │  🧱   │       │       │
  │       │       │ PARED │       │       │
  ├───────┼───────┼───────┼───────┼───────┤
3 │       │🍽ENSAM│       │🏁ENTRE│       │
  ├───────┼───────┼───────┼───────┼───────┤
4 │       │       │       │       │       │
  └───────┴───────┴───────┴───────┴───────┘
```

- Cada celda: 5×5×5m. Paredes de 5m entre celdas.
- 4 robots caben holgados en cualquier celda.
- Arena total: 25×25×5m.

---

## Los 4 robots — Pipeline

| ID | Robot | Rol | Velocidad | Acción principal |
|---|---|---|---|---|
| 0 | 🔵 RECOLECTOR | Pick + deliver | Rápido (10) | ALMACÉN → CORTE |
| 1 | 🔴 CORTADOR | Recibir + cortar | Medio (7) | CORTE [2s] → ENSAMBLAJE |
| 2 | 🟢 ENSAMBLADOR | Armar plato | Medio (7) | ENSAMBLAJE: combina → PLATO |
| 3 | 🟡 REPARTIDOR | Pick + entregar | Rápido (10) | ENSAMBLAJE → ENTREGA ✅ |

---

## Estados de ingredientes

```
CRUDO ──[CORTE 2s]──► CORTADO ──[ENSAMBLAJE]──► PLATO ──[ENTREGA]──► ✅ +150pts
  🥬                      🥬✔️                       🥗
  🍅                      🍅✔️
```

---

## Estructura de archivos

```
RoborregosHack/
├── main.py                      # Orquestador — RoboKitchen class
├── map_config.json              # Layout del mapa (editable)
├── simulation/
│   ├── __init__.py
│   ├── physics.py               # PhysicsWorld (PyBullet DIRECT)
│   ├── robot.py                 # Robot genérico + roles + navegación grid
│   ├── ingredient.py            # Ingrediente con estados + física
│   ├── arena.py                 # Carga map_config.json, construye celdas 3D
│   └── pathfinding.py           # BFS en grid 5×5
├── gemini/
│   ├── __init__.py
│   ├── agent.py                 # GeminiAgent — decide() → comandos
│   └── prompts.py               # System prompt + formateo de estado
├── orders/
│   ├── __init__.py
│   └── order_manager.py         # Tracking de pedidos
├── ui/
│   ├── __init__.py
│   ├── hud.py                   # Pipeline status en pantalla
│   └── scoreboard.py            # Score + game over
├── tests/
│   └── test_physics.py
├── assets/                      # Modelos, texturas, sonidos
├── .env.example
├── .gitignore
├── requirements.txt
└── README.md
```

---

## Contrato Gemini

### Input
```json
{
  "orders": {"pending": 2, "completed": 1},
  "score": 150,
  "robots": [
    {"id": 0, "role": "recolector", "cell": [1,0], "carrying": "lechuga", "action": "moving"},
    {"id": 1, "role": "cortador", "cell": [1,1], "carrying": null, "action": "idle"},
    {"id": 2, "role": "ensamblador", "cell": [1,3], "carrying": null, "action": "waiting"},
    {"id": 3, "role": "repartidor", "cell": [3,3], "carrying": null, "action": "idle"}
  ],
  "ingredients": [
    {"type": "lechuga", "state": "crudo", "cell": [0,0], "held_by": 0},
    {"type": "tomate", "state": "crudo", "cell": [1,0], "held_by": null}
  ],
  "stations": {
    "corte_1": {"cell": [1,1], "processing": false},
    "corte_2": {"cell": [1,2], "processing": false},
    "ensamblaje": {"cell": [1,3], "has_plate": false, "collected": []},
    "entrega": {"cell": [3,3]}
  }
}
```

### Output
```json
[
  {"robot_id": 0, "action": "deliver", "target_cell": [1,1], "reason": "llevar lechuga a corte_1"},
  {"robot_id": 1, "action": "process", "station": "corte_1", "reason": "recibir y cortar"},
  {"robot_id": 2, "action": "wait", "reason": "faltan ingredientes"},
  {"robot_id": 3, "action": "wait", "reason": "no hay plato aun"}
]
```

---

## División del equipo

| Persona | Rama | Archivos |
|---|---|---|
| **P1 — Líder** | `feature/core` | `main.py`, `gemini/*`, `orders/*`, `map_config.json` |
| **P2 — Sim** | `feature/simulation` | `simulation/*`, `tests/*` |
| **P3 — UI/Docs** | `feature/ui` | `ui/*`, `README.md`, `.env.example`, `.gitignore`, `requirements.txt`, video demo |

---

## Timeline — 6 horas

```
HORA 0:00-0:20   SETUP (todos)
                 Clonar, venv, pip install, .env, crear ramas

HORA 0:20-1:50   DESARROLLO PARALELO
                 P1: main.py + gemini + orders
                 P2: simulation/*
                 P3: ui/* + docs

HORA 1:50-2:00   COMMIT INICIAL

HORA 2:00-3:30   PRIMERA INTEGRACIÓN
                 Merge ramas → main. Robots en grid, ingredientes spawnean.
                 Gemini conectado. MVP: recolectar → cortar → ensamblar → entregar.

HORA 3:30-5:00   PIPELINE COMPLETO + POLISH
                 Pipeline funcional. Gemini decide bien. Pathfinding funcional.
                 UI fresca. Probar 5+ ensaladas sin bugs.

HORA 5:00-5:40   VIDEO + DOCS
                 Video 2-3 min demo. README final. Devpost.

HORA 5:40-6:00   ENTREGA
                 Tag v1.0. Subir video. Entregar Devpost.
```

---

## Controles (debug)

| Tecla | Acción |
|---|---|
| WASD | Mover robot recolector |
| R | Reset del juego |
| G | Toggle Gemini ON/OFF |
| ESC | Salir |

---

## Gemini API Key

1. Ir a https://aistudio.google.com/apikey
2. Create API Key
3. Copiar y pegar en `.env`: `GEMINI_API_KEY=tu_key`

Límite gratuito: 15 RPM. El juego llama cada 3s = 20 RPM (ajustar si Gemini falla).
