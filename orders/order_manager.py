"""
orders/order_manager.py — Tracking de pedidos de ensalada.
"""


class OrderManager:

    def __init__(self, points_per_order: int = 150, max_pending: int = 5):
        self.pending = 0
        self.completed = 0
        self.score = 0
        self.points_per_order = points_per_order
        self.max_pending = max_pending
        self.total_spawned = 0

    def spawn(self):
        if self.pending < self.max_pending:
            self.pending += 1
            self.total_spawned += 1
            return True
        return False

    def complete(self):
        if self.pending > 0:
            self.pending -= 1
        self.completed += 1
        self.score += self.points_per_order

    def get_state(self):
        return {
            "pending": self.pending,
            "completed": self.completed,
            "score": self.score,
        }

    def reset(self):
        self.pending = 0
        self.completed = 0
        self.score = 0
        self.total_spawned = 0
