"""
simulation/ingredient.py — Ingrediente con estilo unlit + edge_color.
"""

from ursina import Entity, Vec3, color
from .physics import PhysicsWorld

INGREDIENT_COLORS = {
    "lechuga": {"crudo": color.rgb(60, 190, 60), "cortado": color.rgb(30, 230, 30),
                "plato": color.rgb(255, 210, 60)},
    "tomate":  {"crudo": color.rgb(220, 50, 50), "cortado": color.rgb(250, 30, 30),
                "plato": color.rgb(255, 210, 60)},
}

STATE_SCALE = {"crudo": 0.6, "cortado": 0.45, "plato": 0.85}


class Ingredient:
    MASS = 1.0
    FRICTION = 0.5
    RESTITUTION = 0.4

    def __init__(self, physics: PhysicsWorld, ingredient_type: str,
                 spawn_cell: tuple, world_pos: tuple):
        self.physics = physics
        self.ingredient_type = ingredient_type
        self.state = "crudo"
        self.spawn_cell = spawn_cell
        self.spawn_pos = world_pos
        self.held_by = None
        self.processing = False
        self.process_timer = 0.0

        x, y, z = world_pos
        self.body_id = physics.create_sphere(
            radius=0.3, mass=self.MASS, position=(x, y + 0.5, z),
            friction=self.FRICTION, restitution=self.RESTITUTION)

        self.visual = Entity(
            model="sphere",
            color=INGREDIENT_COLORS[ingredient_type]["crudo"],
            scale=STATE_SCALE["crudo"],
            position=Vec3(x, y + 0.5, z),
            unlit=True, edge_color=color.black, edge_width=2,
        )

    def sync_visual(self):
        if self.held_by is not None:
            return
        pos = self.physics.get_position(self.body_id)
        self.visual.position = Vec3(pos[0], pos[1], pos[2])

    def get_position(self):
        pos = self.physics.get_position(self.body_id)
        return Vec3(pos[0], pos[1], pos[2])

    def set_state(self, new_state: str):
        if new_state not in ("crudo", "cortado", "plato"):
            return
        self.state = new_state
        self.visual.color = INGREDIENT_COLORS[self.ingredient_type][new_state]
        self.visual.scale = STATE_SCALE[new_state]
        if new_state == "plato":
            self.visual.model = "cube"

    def pickup(self, robot):
        self.held_by = robot
        self.visual.position = robot.visual.position + Vec3(0, 1.5, 0)
        self.visual.parent = robot.visual

    def drop(self, world_pos):
        self.held_by = None
        self.visual.parent = None
        self.physics.reset_body(self.body_id,
                                (world_pos.x, world_pos.y + 0.3, world_pos.z))
        self.visual.position = Vec3(world_pos.x, world_pos.y + 0.3, world_pos.z)

    def reset(self):
        self.state = "crudo"
        self.held_by = None
        self.processing = False
        self.process_timer = 0.0
        self.visual.parent = None
        self.visual.color = INGREDIENT_COLORS[self.ingredient_type]["crudo"]
        self.visual.scale = STATE_SCALE["crudo"]
        self.visual.model = "sphere"
        x, y, z = self.spawn_pos
        self.physics.reset_body(self.body_id, (x, y + 0.5, z))
        self.visual.position = Vec3(x, y + 0.5, z)
