"""
gemini/agent.py — GeminiAgent: chef supervisor del pipeline.
Recibe estado del juego, retorna plan de acciones para los 4 robots.
"""

import os
import json
import google.generativeai as genai
from dotenv import load_dotenv
from .prompts import SYSTEM_PROMPT, format_game_state

load_dotenv()

VALID_ACTIONS = {"goto", "pickup", "deliver", "process", "assemble",
                 "pickup_plate", "wait", "idle"}


class GeminiAgent:

    MODEL_NAME = "gemini-2.0-flash"

    def __init__(self):
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise EnvironmentError(
                "GEMINI_API_KEY no encontrada. Crea .env con: GEMINI_API_KEY=tu_key")

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel(
            model_name=self.MODEL_NAME,
            system_instruction=SYSTEM_PROMPT,
        )
        self._call_count = 0
        print(f"[GeminiAgent] modelo={self.MODEL_NAME} | OK")

    def decide(self, game_state: dict) -> list[dict]:
        self._call_count += 1

        prompt = (
            f"=== Estado del juego (llamada #{self._call_count}) ===\n"
            f"{format_game_state(game_state)}\n\n"
            f"Decide las acciones optimas para los 4 robots. "
            f"Responde UNICAMENTE con el array JSON."
        )

        try:
            response = self.model.generate_content(
                prompt,
                generation_config=genai.GenerationConfig(
                    temperature=0.2,
                    max_output_tokens=300,
                ),
            )
            return self._parse_response(response.text)
        except Exception as e:
            print(f"[GeminiAgent] Error #{self._call_count}: {e}")
            return self._fallback_commands(game_state)

    def _parse_response(self, text: str) -> list[dict]:
        text = text.strip()
        if text.startswith("```"):
            parts = text.split("```")
            text = parts[1] if len(parts) >= 2 else text
            if text.startswith("json"):
                text = text[4:]
        text = text.strip()

        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            import re
            match = re.search(r"\[.*\]", text, re.DOTALL)
            if match:
                data = json.loads(match.group())
            else:
                print(f"[GeminiAgent] JSON invalido: {text[:120]}")
                return []

        validated = []
        for cmd in data:
            if not isinstance(cmd, dict):
                continue
            validated.append({
                "robot_id": int(cmd.get("robot_id", 0)),
                "action": cmd.get("action", "idle"),
                "target_cell": cmd.get("target_cell"),
                "target_station": cmd.get("target_station"),
                "station": cmd.get("station"),
                "reason": cmd.get("reason", ""),
            })
        return validated if validated else []

    def _fallback_commands(self, game_state: dict) -> list[dict]:
        """Pipeline hardcodeado: asigna cada robot al siguiente paso logico."""
        cmds = []

        robots = {r["id"]: r for r in game_state.get("robots", [])}
        orders = game_state.get("orders", {})
        stations = game_state.get("stations", {})
        ingredients = game_state.get("ingredients", [])

        pending = orders.get("pending", 0)
        has_plate = stations.get("ensamblaje", {}).get("has_plate", False)

        recolector = robots.get(0, {})
        cortador = robots.get(1, {})
        ensamblador = robots.get(2, {})
        repartidor = robots.get(3, {})

        if pending > 0:
            crudos = [i for i in ingredients if i.get("state") == "crudo" and not i.get("held_by")]
            if crudos and not recolector.get("carrying"):
                ing = crudos[0]
                almacen = [c for c in game_state.get("cells", []) if c.get("type") == "almacen"]
                target = (ing.get("row", 0), ing.get("col", 0))
                cmds.append({"robot_id": 0, "action": "pickup", "target_cell": target,
                             "reason": f"recoger {ing['type']} crudo"})
                cortes = [c for c in game_state.get("cells", []) if c.get("type") == "corte"]
                if cortes:
                    cmds.append({"robot_id": 1, "action": "goto",
                                 "target_cell": (cortes[0]["row"], cortes[0]["col"]),
                                 "reason": "ir a corte"})
                    cmds.append({"robot_id": 2, "action": "goto",
                                 "target_cell": (3, 1), "reason": "ir a ensamblaje"})
                    cmds.append({"robot_id": 3, "action": "goto",
                                 "target_cell": (3, 3), "reason": "ir a entrega"})
            elif recolector.get("carrying"):
                cortes = [c for c in game_state.get("cells", []) if c.get("type") == "corte"]
                if cortes:
                    cmds.append({"robot_id": 0, "action": "deliver",
                                 "target_cell": (cortes[0]["row"], cortes[0]["col"]),
                                 "reason": "entregar en corte"})

        if has_plate:
            cmds.append({"robot_id": 3, "action": "pickup_plate",
                         "target_cell": [3, 1], "reason": "recoger plato"})

        while len(cmds) < 4:
            rid = len(cmds)
            cmds.append({"robot_id": rid, "action": "idle", "reason": "sin tarea"})

        return cmds[:4]
