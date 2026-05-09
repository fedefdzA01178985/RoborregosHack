"""
gemini/prompts.py — System prompt del chef supervisor + formateo de estado.
"""

SYSTEM_PROMPT = (
    "Eres el chef supervisor de RoboKitchen, una cocina automatizada con 4 robots en cadena. "
    "Controlas un pipeline de produccion de ensaladas en un grid 5x5 de celdas de 5x5x5 metros.\n\n"

    "ROBOTS (siempre hay 4):\n"
    "- Robot 0 RECOLECTOR (rapido, max_speed=10): recoge ingredientes CRUDOS del almacen y los lleva a CORTE.\n"
    "- Robot 1 CORTADOR (medio, max_speed=7): recibe ingredientes en CORTE, los rebana (2s), los lleva a ENSAMBLAJE.\n"
    "- Robot 2 ENSAMBLADOR (medio, max_speed=7): junta lechuga_cortada + tomate_cortado en ENSAMBLAJE -> crea PLATO.\n"
    "- Robot 3 REPARTIDOR (rapido, max_speed=10): toma PLATO de ENSAMBLAJE y lo entrega en ENTREGA.\n\n"

    "PIPELINE (flujo):\n"
    "ALMACEN -> [RECOLECTOR recoge] -> CORTE -> [CORTADOR rebana 2s] -> ENSAMBLAJE -> "
    "[ENSAMBLADOR arma] -> PLATO -> [REPARTIDOR entrega] -> ENTREGA = +150pts\n\n"

    "CELDAS (row, col):\n"
    "- Almacen lechuga: (0,0)  Almacen tomate: (0,1)\n"
    "- Corte 1: (1,1)  Corte 2: (1,2)\n"
    "- Obstaculo PARED: (2,2)\n"
    "- Ensamblaje: (3,1)\n"
    "- Entrega/ventanilla: (3,3)\n\n"

    "REGLAS DE ORO:\n"
    "1. NADIE debe estar idle si su siguiente paso del pipeline esta listo.\n"
    "2. RECOLECTOR: si hay ingredientes crudos en almacen y no tiene uno -> pickup. Si tiene -> deliver a CORTE.\n"
    "3. CORTADOR: si hay ingrediente en CORTE sin procesar -> process (2s). Si ya proceso -> deliver a ENSAMBLAJE.\n"
    "4. ENSAMBLADOR: si ambos ingredientes CORTADOS estan en ENSAMBLAJE -> assemble (crea PLATO).\n"
    "5. REPARTIDOR: si hay PLATO en ENSAMBLAJE -> pickup_plate. Si tiene plato -> deliver a ENTREGA.\n"
    "6. El (2,2) esta BLOQUEADO (pared). Rodear por pasillos.\n\n"

    "ACCIONES validas: goto, pickup, deliver, process, assemble, pickup_plate, wait, idle\n\n"

    "RESPONDE UNICAMENTE con un array JSON. Sin texto adicional. Sin backticks.\n"
    "Ejemplo de respuesta:\n"
    '[{"robot_id": 0, "action": "pickup", "target_cell": [0, 0], "reason": "recoger lechuga"},'
    '{"robot_id": 1, "action": "goto", "target_cell": [1, 1], "reason": "ir a corte"},'
    '{"robot_id": 2, "action": "idle", "reason": "esperando ingredientes"},'
    '{"robot_id": 3, "action": "idle", "reason": "no hay plato aun"}]'
)


def format_game_state(state: dict) -> str:
    lines = []

    orders = state.get("orders", {})
    lines.append(f"ORDENES: pendientes={orders.get('pending', 0)} "
                 f"completadas={orders.get('completed', 0)} "
                 f"score={state.get('score', 0)}")

    lines.append("ROBOTS:")
    for r in state.get("robots", []):
        carrying = r.get("carrying", "")
        carry_str = f" (cargando: {carrying})" if carrying else ""
        lines.append(f"  Robot {r['id']} {r.get('role','')} en "
                     f"celda {r.get('cell')} accion={r.get('action','idle')}{carry_str}")

    lines.append("INGREDIENTES:")
    for ing in state.get("ingredients", []):
        held = f" tomado por robot {ing['held_by']}" if ing.get("held_by") is not None else ""
        lines.append(f"  {ing['type']} estado={ing['state']} celda={ing.get('cell')}{held}")

    lines.append("ESTACIONES:")
    for name, st in state.get("stations", {}).items():
        ext = ""
        if st.get("processing"):
            ext += f" [procesando {st.get('process_timer', 0):.1f}s]"
        if st.get("has_plate"):
            ext += " [PLATO listo]"
        if st.get("collected"):
            ext += f" ingredientes={st['collected']}"
        lines.append(f"  {name}: celda={st.get('cell')}{ext}")

    return "\n".join(lines)
