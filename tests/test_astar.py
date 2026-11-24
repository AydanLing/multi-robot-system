# tests/test_astar.py
from fleet.world import World
from fleet.planning.astar import astar

OPEN = [".....",
        ".....",
        "....."]

WALL = [".#...",
        ".#.#.",
        "...#."]

def is_valid_path(world, path, start, goal):
    """Starts and ends correctly, only free cells, one step at a time."""
    if path[0] != start or path[-1] != goal:
        return False
    for a, b in zip(path, path[1:]):
        if b not in world.neighbours(a):
            return False
    return True

def test_straight_line():
    w = World(OPEN)
    path = astar(w, (0, 0), (0, 4))
    assert len(path) == 5                      # 4 moves + start
    assert is_valid_path(w, path, (0, 0), (0, 4))

def test_goes_around_walls():
    w = World(WALL)
    path = astar(w, (0, 0), (0, 2))
    assert is_valid_path(w, path, (0, 0), (0, 2))
    assert len(path) == 7                      # down, down, right, right, up, up

def test_start_equals_goal():
    assert astar(World(OPEN), (1, 1), (1, 1)) == [(1, 1)]

def test_unreachable_returns_none():
    w = World(["..#..",
               "..#..",
               "..#.."])
    assert astar(w, (0, 0), (0, 4)) is None

def test_goal_on_shelf_returns_none():
    assert astar(World(WALL), (0, 0), (0, 1)) is None