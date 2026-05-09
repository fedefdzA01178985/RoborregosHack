"""
simulation/robot.py — Robot con estilo unlit + edge_color (estilo dibujo).
"""

from ursina import Entity, Vec3, color
from .physics import PhysicsWorld

ROLE_CONFIG = {
    "recolector":   {"max_speed": 10.0, "force": 25.0},
    "cortador":     {"max_speed":  7.0, "force": 18.0},
    "ensamblador":  {"max_speed":  7.0, "force": 18.0},
    "repartidor":   {"max_speed": 10.0, "force": 25.0},
}

ROLE_COLORS = {
    "recolector":   color.rgb(50, 120, 240),
    "cortador":     color.rgb(240, 60, 60),
    "ensamblador":  color.rgb(50, 210, 60),
    "repartidor":   color.rgb(240, 210, 30),
}


class Robot:
    HALF_EXTENTS = (0.5, 0.4, 0.6)
    URSINA_SCALE = (1.0, 0.8, 1.2)
    MASS = 3.0
    FRICTION = 1.2
    RESTITUTION = 0.1

    def __init__(self, physics: PhysicsWorld, robot_id: int, role: str,
                 start_cell: tuple, world_pos: tuple):
        self.physics = physics
        self.robot_id = robot_id
        self.role = role
        self.start_cell = start_cell
        self.start_pos = world_pos
        self.config = ROLE_CONFIG[role]
        self.current_cell = start_cell
        self.carrying = None
        self.action = "idle"
        self.stuck_timer = 0.0

        x, y, z = world_pos
        self.body_id = physics.create_box(
            half_extents=self.HALF_EXTENTS, mass=self.MASS,
            position=(x, y + 0.5, z), friction=self.FRICTION,
            restitution=self.RESTITUTION, linear_damping=0.5,
            angular_damping=0.9)

        self.visual = Entity(
            model="cube", color=ROLE_COLORS[role],
            scale=self.URSINA_SCALE, position=Vec3(x, y + 0.5, z),
            unlit=True, edge_color=color.black, edge_width=3,
        )

        self.plate = Entity(
            parent=self.visual, model="cube",
            color=color.rgb(255, 220, 80),
            scale=(0.7, 0.15, 0.7), position=Vec3(0, 1.2, 0),
            unlit=True, edge_color=color.black, edge_width=2,
            enabled=False,
        )

    def sync_visual(self):
        pos = self.physics.get_position(self.body_id)
        self.visual.position = Vec3(pos[0], pos[1], pos[2])

    def get_position(self):
        pos = self.physics.get_position(self.body_id)
        return Vec3(pos[0], pos[1], pos[2])

    def move_toward(self, target_world_pos):
        pos = self.get_position()
        dx = target_world_pos.x - pos.x
        dz = target_world_pos.z - pos.z
        dist = (dx*dx + dz*dz) ** 0.5
        if dist < 0.3:
            self._stop()
            return
        force_x = (dx / dist) * self.config["force"]
        force_z = (dz / dist) * self.config["force"]
        self.physics.apply_force(self.body_id, (force_x, 0, force_z))
        self._clamp_speed()

    def pickup(self, ingredient):
        self.carrying = ingredient
        ingredient.pickup(self)
        self.action = "carrying"
        self.plate.enabled = True

    def drop_at(self, world_pos):
        if self.carrying:
            self.carrying.drop(world_pos)
            self.carrying = None
        self.action = "idle"
        self.plate.enabled = False

    def _stop(self):
        self.physics.reset_body(self.body_id, self.get_position())

    def _clamp_speed(self):
        import pybullet as p
        lin = self.physics.get_velocity(self.body_id)
        speed = (lin[0]**2 + lin[2]**2) ** 0.5
        if speed > self.config["max_speed"]:
            factor = self.config["max_speed"] / speed
            p.resetBaseVelocity(self.body_id,
                linearVelocity=(lin[0]*factor, lin[1], lin[2]*factor),
                angularVelocity=(0, 0, 0),
                physicsClientId=self.physics.client)

    def reset(self, world_pos=None):
        target = world_pos if world_pos else self.start_pos
        self.carrying = None
        self.action = "idle"
        self.current_cell = self.start_cell
        self.stuck_timer = 0.0
        self.plate.enabled = False
        self.physics.reset_body(self.body_id, (target[0], target[1] + 0.5, target[2]))
        self.visual.position = Vec3(target[0], target[1] + 0.5, target[2])
