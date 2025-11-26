from maze import Maze, Direction


def build_competition_maze(m: Maze) -> Maze:
    # Inner box.
    for x in range(1, 5):
        m.add_wall(x, 0, Direction.N)
    for y in range(1, 7):
        m.add_wall(0, y, Direction.E)
    m.add_wall(1, 6, Direction.N)
    m.add_wall(3, 6, Direction.N)
    m.add_wall(4, 6, Direction.N)

    # Inner elements.
    m.add_wall(1, 5, Direction.S)
    m.add_wall(1, 5, Direction.E)
    m.add_wall(1, 4, Direction.S)
    m.add_wall(1, 3, Direction.S)
    m.add_wall(1, 2, Direction.S)
    m.add_wall(2, 5, Direction.E)
    m.add_wall(2, 2, Direction.E)
    m.add_wall(2, 1, Direction.E)
    m.add_wall(3, 2, Direction.E)
    m.add_wall(4, 6, Direction.S)
    m.add_wall(4, 5, Direction.S)
    m.add_wall(4, 4, Direction.S)
    m.add_wall(4, 2, Direction.S)

    return m


def ramp_to_drop() -> Maze:
    width, length = 5, 8
    start = (4, 0)
    goal = (1, 4)
    ramp_to_drop = Maze(width, length, start, goal)
    return build_competition_maze(ramp_to_drop)


def drop_to_pick() -> Maze:
    width, length = 5, 8
    start = (1, 4)
    goal = (4, 6)
    ramp_to_drop = Maze(width, length, start, goal)
    return build_competition_maze(ramp_to_drop)


def pick_to_lobby() -> Maze:
    width, length = 5, 8
    start = (4, 6)
    goal = (4, 0)
    ramp_to_drop = Maze(width, length, start, goal)
    return build_competition_maze(ramp_to_drop)


def main():
    maze = ramp_to_drop()
    maze.draw()

    maze = drop_to_pick()
    maze.draw()

    maze = pick_to_lobby()
    maze.draw()


if __name__ == "__main__":
    main()
