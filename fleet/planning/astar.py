import heapq
from itertools import count

from fleet.world import World, Cell


def manhattan(a: Cell, b: Cell) -> int: # the heuristic
    # grid distance ignoring obstacles: rows apart + columns apart
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(world: World, start: Cell, goal: Cell) -> list[Cell] | None:
    # shortest path from start to goal, including both ends, and none if the goal is unreachable
    if not world.is_free(start) or not world.is_free(goal):
        return None

    tie = count()
    open_heap = [(manhattan(start, goal), next(tie), start)]
    came_from: dict[Cell, Cell] = {} # the cell we reached it from
    g = {start: 0} # the best known cost to reach each cell
    closed: set[Cell] = set() # the set of cells already explored

    while open_heap:
        _, _, current = heapq.heappop(open_heap) # cell with the lowest f

        if current == goal:
            return reconstruct_path(came_from, current)

        if current in closed:
            continue
        closed.add(current)

        for nxt in world.neighbours(current):
            new_g = g[current]+1
            if new_g < g.get(nxt, float("inf")):
                g[nxt] = new_g
                came_from[nxt] = current
                f = new_g + manhattan(nxt, goal)
                heapq.heappush(open_heap, (f, next(tie), nxt))
    return None

def reconstruct_path(came_from: dict[Cell, Cell], current: Cell) -> list[Cell]:
    # follow the links backwards from the goal, then reverse

    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
