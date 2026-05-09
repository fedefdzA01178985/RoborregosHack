"""
ui/hud.py — HUD de pipeline status en pantalla (Ursina Text).
"""

from ursina import Text, Vec2, color


class HUD:

    def __init__(self):
        self.fps_text = Text(
            text="FPS: --", position=Vec2(-0.85, 0.48),
            scale=1.1, color=color.lime)

        self.score_text = Text(
            text="Score: 0", position=Vec2(-0.85, 0.42),
            scale=1.0, color=color.yellow)

        self.orders_text = Text(
            text="Pedidos: --", position=Vec2(-0.85, 0.37),
            scale=0.9, color=color.cyan)

        self.gemini_text = Text(
            text="Gemini: ON", position=Vec2(-0.85, 0.32),
            scale=0.9, color=color.green)

        self.robot_texts = []
        for i in range(4):
            t = Text(
                text=f"R{i}: --", position=Vec2(-0.85, 0.26 - i * 0.05),
                scale=0.8, color=color.white)
            self.robot_texts.append(t)

    def update(self, fps: int, score: int, pending: int, completed: int,
               gemini_enabled: bool, robots: list, stations: dict):
        self.fps_text.text = f"FPS: {fps}"
        self.score_text.text = f"Score: {score}"
        self.orders_text.text = f"Pedidos: {pending} pend | {completed} listos"

        gem_color = color.green if gemini_enabled else color.red
        self.gemini_text.text = f"Gemini: {'ON' if gemini_enabled else 'OFF'}"
        self.gemini_text.color = gem_color

        roles_map = {0: "REC", 1: "COR", 2: "ENS", 3: "REP"}
        for i, rt in enumerate(self.robot_texts):
            if i < len(robots):
                r = robots[i]
                label = roles_map.get(i, "R?")
                cell = r.get("cell", "?")
                action = r.get("action", "?")[:8]
                carry = "+" if r.get("carrying") else "-"
                rt.text = f"{label}: c{cell} {action} [{carry}]"
                rt.color = r.get("color", color.white)
            else:
                rt.text = f"R{i}: --"
