"""
simulation/pathfinding.py — BFS simple en grid 5x5.
Evita celdas bloqueadas (obstacles) y paredes entre celdas.
"""

from collections import deque


class PathFinder:

    def __init__(self, rows: int = 5, cols: int = 5):
        self.rows = rows
        self.cols = cols

    def find_path(self, grid, start, goal):
        """
        grid: matriz rows x cols de bool (True = transitable)
        start, goal: (row, col)
        Retorna: list[(row, col)] o None si no hay ruta
        """
        if start == goal:
            return [start]

        visited = [[False] * self.cols for _ in range(self.rows)]
        parent = [[None] * self.cols for _ in range(self.rows)]
        queue = deque()
        queue.append(start)
        visited[start[0]][start[1]] = True

        while queue:
            r, c = queue.popleft()
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if not visited[nr][nc] and grid[nr][nc]:
                        visited[nr][nc] = True
                        parent[nr][nc] = (r, c)
                        if (nr, nc) == goal:
                            return self._reconstruct(parent, start, goal)
                        queue.append((nr, nc))
        return None

    def _reconstruct(self, parent, start, goal):
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = parent[current[0]][current[1]]
        path.append(start)
        path.reverse()
        return path
