"""
ui/scoreboard.py — Marcador de puntaje y pantalla de game over.
"""

from ursina import Text, Vec2, color


class Scoreboard:

    def __init__(self):
        self.title = Text(
            text="RoboKitchen", position=Vec2(0, 0.48),
            origin=(0, 0), scale=2.2, color=color.white, bold=True)

        self.score_display = Text(
            text="0 pts", position=Vec2(0, 0.43),
            origin=(0, 0), scale=1.5, color=color.yellow)

        self.order_counter = Text(
            text="0 ensaladas", position=Vec2(0, 0.39),
            origin=(0, 0), scale=0.9, color=color.cyan)

        self._game_over_text = None

    def update(self, score: int, completed: int):
        self.score_display.text = f"{score} pts"
        self.order_counter.text = f"{completed} ensaladas"

    def show_game_over(self, score: int, completed: int):
        self._game_over_text = Text(
            text=f"GAME OVER\n{score} pts | {completed} ensaladas",
            position=Vec2(0, 0), origin=(0, 0), scale=3.0,
            color=color.rgb(255, 200, 50), background=True)
