# Persona 2 — Simulacion

## Tus archivos

```
simulation/
├── __init__.py        (ya creado, no tocar)
├── physics.py         (ya creado)
├── robot.py           (ya creado)
├── ingredient.py      (ya creado)
├── arena.py           (ya creado)
└── pathfinding.py     (ya creado)

tests/
└── test_physics.py    (ya creado)
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

# 3. Probar fisica
python tests\test_physics.py
# Debe decir: [OK] Todos los tests de fisica pasaron
```

## Que necesitas hacer

Tu trabajo es completar/intregrar estos archivos. Usa tu IA para generar el codigo. Tu supervisas que funcione.

### simulation/physics.py
- **Estado:** COMPLETO
- Wrapper PyBullet en modo DIRECT. `create_box()`, `create_sphere()`, `step()`, `apply_force()`, etc.
- NO necesita cambios a menos que detectes bugs.

### simulation/robot.py
- **Estado:** COMPLETO
- 4 robots con roles: recolector, cortador, ensamblador, repartidor.
- `move_toward()`, `pickup()`, `drop_at()`, `sync_visual()`, `reset()`.
- Verifica que los robots NO se vuelquen (angular_damping=0.9).

### simulation/ingredient.py
- **Estado:** COMPLETO
- Ingrediente con estados: crudo → cortado → plato.
- `pickup()`, `drop()`, `set_state()`, `sync_visual()`, `reset()`.
- Ajusta colores y escalas visuales en STATE_COLORS / STATE_SCALE.

### simulation/arena.py
- **Estado:** COMPLETO (puede necesitar ajustes)
- Carga `map_config.json` y construye el grid 3D.
- Metodos clave: `cell_center()`, `is_passable()`, `get_cell_at()`, `get_cell_type()`.
- **TU TAREA:** Probar que las paredes y pisos se ven bien en 3D. Ajustar colores.

### simulation/pathfinding.py
- **Estado:** COMPLETO
- BFS en grid 5x5. `find_path(grid, start, goal)` → lista de celdas.
- NO necesita cambios.

### tests/test_physics.py
- **Estado:** COMPLETO
- Test basico: crear caja, aplicar fuerza, verificar que se movio.

## Lo que NO debes tocar

- `main.py` — es de Persona 1
- `gemini/` — es de Persona 1
- `orders/` — es de Persona 1
- `ui/` — es de Persona 3
- `map_config.json` — Persona 1 puede necesitar ajustarlo

## Git workflow

```bash
git checkout -b feature/simulation
# ... trabajas en tus archivos ...
git add simulation/ tests/
git commit -m "feat: simulacion completa"
git push -u origin feature/simulation
# Crear PR en GitHub: base main ← feature/simulation
```

## Si necesitas probar con el juego completo

```bash
# Despues de que Persona 1 haga merge de su rama:
git checkout main
git pull origin main
git checkout feature/simulation
git merge main
python main.py
```
