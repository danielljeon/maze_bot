from collections import deque
from enum import Enum

import matplotlib.pyplot as plt


class Direction(str, Enum):
    N = "N"
    E = "E"
    S = "S"
    W = "W"


OPPOSITE = {
    Direction.N: Direction.S,
    Direction.S: Direction.N,
    Direction.E: Direction.W,
    Direction.W: Direction.E,
}

DIR_DELTAS = {
    Direction.N: (0, 1),
    Direction.S: (0, -1),
    Direction.E: (1, 0),
    Direction.W: (-1, 0),
}


class Maze:
    """
    Cartesian-style thin-wall maze.

    - Cells are at integer coordinates (x, y).
      x = 0..width-1,  y = 0..height-1
      (0,0) is bottom-left corner.

    - Each cell has booleans for walls on N/E/S/W edges.

    - Walls are thin (on edges between cells).
    """

    def __init__(self, width, height, start, goal):
        self.width = width
        self.height = height
        self.start = start  # (x, y).
        self.goal = goal  # (x, y).

        # walls[x][y] = { "N": bool, "E": bool, "S": bool, "W": bool }.
        self.walls = [
            [{d: False for d in Direction} for _y in range(height)]
            for _x in range(width)
        ]

        # Closed outer loop around the whole field.
        for x in range(width):
            self.add_wall(x, 0, Direction.S)  # Bottom.
            self.add_wall(x, height - 1, Direction.N)  # Top.
        for y in range(height):
            self.add_wall(0, y, Direction.W)  # Left.
            self.add_wall(width - 1, y, Direction.E)  # Right.

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def add_wall(self, x, y, direction: Direction):
        """Add a wall on cell (x,y) in the given direction.

        Also mirrors it onto the neighbor cell (if in bounds).
        """
        if not self.in_bounds(x, y):
            return

        self.walls[x][y][direction] = True

        dx, dy = {
            Direction.N: (0, 1),
            Direction.S: (0, -1),
            Direction.E: (1, 0),
            Direction.W: (-1, 0),
        }[direction]

        nx, ny = x + dx, y + dy
        if self.in_bounds(nx, ny):
            self.walls[nx][ny][OPPOSITE[direction]] = True

    def neighbors(self, x, y):
        """
        Returns reachable neighboring cells from (x,y),
        respecting walls. Only 4-connected (N/E/S/W) moves.
        """
        result = []
        for d, (dx, dy) in DIR_DELTAS.items():
            if not self.walls[x][y][d]:  # no wall blocking this direction
                nx, ny = x + dx, y + dy
                if self.in_bounds(nx, ny):
                    result.append((nx, ny))
        return result

    def __bfs_shortest_path(self):
        """Breadth-first search for shortest path (fewest steps).

        Starts from maze.start to  maze.goal, using only N/E/S/W moves.

        Returns:
             List of (x,y) cells including start and goal, or None if no path.
        """
        start = self.start
        goal = self.goal

        queue = deque([start])
        came_from = {start: None}

        while queue:
            x, y = queue.popleft()
            if (x, y) == goal:
                break

            for nx, ny in self.neighbors(x, y):
                if (nx, ny) not in came_from:
                    came_from[(nx, ny)] = (x, y)
                    queue.append((nx, ny))

        if goal not in came_from:
            return None  # no path

        # Reconstruct path from goal to start
        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()
        return path

    def get_path(self):
        return self.__bfs_shortest_path()

    def draw(self, path: bool = True):
        fig, ax = plt.subplots()
        ax.set_aspect("equal")

        # Light cell grid
        for x in range(self.width + 1):
            ax.plot([x, x], [0, self.height], color="lightgray", linewidth=0.4)
        for y in range(self.height + 1):
            ax.plot([0, self.width], [y, y], color="lightgray", linewidth=0.4)

        # Draw walls
        for x in range(self.width):
            for y in range(self.height):
                w = self.walls[x][y]
                if w[Direction.N]:
                    ax.plot(
                        [x, x + 1], [y + 1, y + 1], color="black", linewidth=3
                    )
                if w[Direction.S]:
                    ax.plot([x, x + 1], [y, y], color="black", linewidth=3)
                if w[Direction.E]:
                    ax.plot(
                        [x + 1, x + 1], [y, y + 1], color="black", linewidth=3
                    )
                if w[Direction.W]:
                    ax.plot([x, x], [y, y + 1], color="black", linewidth=3)

        # Start and goal.
        sx, sy = self.start
        gx, gy = self.goal
        ax.scatter(sx + 0.5, sy + 0.5, s=120, color="green", label="start")
        ax.scatter(
            gx + 0.5, gy + 0.5, s=120, color="red", marker="x", label="goal"
        )

        if path:
            computed_path = self.get_path()
            xs = [x + 0.5 for (x, y) in computed_path]
            ys = [y + 0.5 for (x, y) in computed_path]
            ax.plot(xs, ys, linewidth=3, color="blue")

        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)
        ax.set_xticks(range(self.width + 1))
        ax.set_yticks(range(self.height + 1))
        ax.set_xlabel("x (cells)")
        ax.set_ylabel("y (cells)")
        ax.set_title("Maze")
        ax.legend(loc="upper right")
        plt.show()
