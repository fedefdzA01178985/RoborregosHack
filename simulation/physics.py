"""
simulation/physics.py — Wrapper PyBullet en modo DIRECT.
Ursina hace el render; PyBullet solo calcula.
"""

import pybullet as p
import pybullet_data


class PhysicsWorld:
    GRAVITY = (0, -9.81, 0)
    STEP_TIME = 1.0 / 240.0

    def __init__(self):
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(),
                                  physicsClientId=self.client)
        p.setGravity(*self.GRAVITY, physicsClientId=self.client)
        p.setTimeStep(self.STEP_TIME, physicsClientId=self.client)
        p.setPhysicsEngineParameter(
            numSolverIterations=50,
            numSubSteps=4,
            physicsClientId=self.client,
        )
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client)
        print(f"[PhysicsWorld] client={self.client} | gravity={self.GRAVITY}")

    def step(self):
        p.stepSimulation(physicsClientId=self.client)

    def create_box(self, half_extents, mass, position, friction=0.8,
                   restitution=0.3, linear_damping=0.1, angular_damping=0.1):
        col_shape = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=half_extents,
            physicsClientId=self.client)
        vis_shape = p.createVisualShape(
            p.GEOM_BOX, halfExtents=half_extents,
            physicsClientId=self.client)
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=col_shape,
            baseVisualShapeIndex=vis_shape,
            basePosition=position,
            physicsClientId=self.client)
        p.changeDynamics(body_id, -1,
                         lateralFriction=friction,
                         restitution=restitution,
                         linearDamping=linear_damping,
                         angularDamping=angular_damping,
                         physicsClientId=self.client)
        return body_id

    def create_sphere(self, radius, mass, position, friction=0.5, restitution=0.7):
        col_shape = p.createCollisionShape(
            p.GEOM_SPHERE, radius=radius,
            physicsClientId=self.client)
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=col_shape,
            basePosition=position,
            physicsClientId=self.client)
        p.changeDynamics(body_id, -1,
                         lateralFriction=friction,
                         restitution=restitution,
                         linearDamping=0.05,
                         angularDamping=0.05,
                         physicsClientId=self.client)
        return body_id

    def get_position(self, body_id):
        pos, _ = p.getBasePositionAndOrientation(body_id,
                                                  physicsClientId=self.client)
        return pos

    def get_orientation(self, body_id):
        _, orn = p.getBasePositionAndOrientation(body_id,
                                                  physicsClientId=self.client)
        return orn

    def get_velocity(self, body_id):
        lin, _ = p.getBaseVelocity(body_id, physicsClientId=self.client)
        return lin

    def apply_force(self, body_id, force):
        p.applyExternalForce(body_id, -1, forceObj=force,
                             posObj=(0, 0, 0), flags=p.WORLD_FRAME,
                             physicsClientId=self.client)

    def reset_body(self, body_id, position, orientation=(0, 0, 0, 1)):
        p.resetBasePositionAndOrientation(body_id, posObj=position,
                                          ornObj=orientation,
                                          physicsClientId=self.client)
        p.resetBaseVelocity(body_id, linearVelocity=(0, 0, 0),
                            angularVelocity=(0, 0, 0),
                            physicsClientId=self.client)

    def remove_body(self, body_id):
        p.removeBody(body_id, physicsClientId=self.client)

    def __del__(self):
        try:
            p.disconnect(physicsClientId=self.client)
        except Exception:
            pass
