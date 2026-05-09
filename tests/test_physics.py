"""
tests/test_physics.py — Test basico de colisiones PyBullet sin UI.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation import PhysicsWorld


def test_create_and_move():
    pw = PhysicsWorld()
    body = pw.create_box(
        half_extents=(0.5, 0.5, 0.5),
        mass=1.0,
        position=(0, 1, 0),
    )
    pw.step()
    pos = pw.get_position(body)
    assert pos[1] > 0, f"Expected y>0, got {pos}"
    print("  [PASS] create_and_move")

    pw.apply_force(body, (10, 0, 0))
    for _ in range(60):
        pw.step()
    pos2 = pw.get_position(body)
    assert pos2[0] > pos[0], f"Expected x movement, {pos} -> {pos2}"
    print("  [PASS] apply_force")

    pw.remove_body(body)
    print("  [PASS] remove_body")
    print("[OK] Todos los tests de fisica pasaron")


if __name__ == "__main__":
    test_create_and_move()
