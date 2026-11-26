import heapq
from itertools import count

from fleet.planning.astar import manhattan
from fleet.planning.reservations import ReservationTable
from fleet.world import World, Cell

State = tuple[Cell, int]  # (cell, time)


def space_time_astar(
    world: World,
    start: Cell,
    goal: Cell,
    start_t: int,
    table: ReservationTable,
    max_steps: int | None = None,
    stay: bool = True,
    dwell: int = 0,
) -> list[Cell] | None:
    """Shortest path that avoids everything booked in the table.

    Returns the cell at each time step: path[i] is where the robot is at start_t + i.
    A repeated cell means the robot waits. Returns None if no path is found within max_steps.
    stay=True: the robot must be able to stay at the goal for good (parking).
    stay=False: it only needs the goal for `dwell` steps after arriving (pickup/drop-off).
    """
    if not world.is_free(start) or not world.is_free(goal):
        return None
    if stay and goal in table.parked:
        return None          # someone is parked on the goal for good: don't waste time searching
    if not stay and table.parked.get(goal, start_t + 1) <= start_t:
        return None          # someone is already sitting on the goal for good
    if max_steps is None:
        max_steps = 2 * world.width * world.height   # waiting is always possible, so the search needs a limit

    tie = count()
    start_state: State = (start, start_t)
    open_heap = [(manhattan(start, goal), next(tie), start_state)]
    came_from: dict[State, State] = {}
    seen: set[State] = {start_state}

    while open_heap:
        _, _, (cell, t) = heapq.heappop(open_heap)

        if cell == goal and (table.can_park(goal, t) if stay else table.can_visit(goal, t, dwell)):
            return _reconstruct(came_from, (cell, t))

        if t - start_t >= max_steps:
            continue

        for nxt in world.neighbours(cell) + [cell]:      # 4 moves, plus waiting in place
            nt = t + 1
            if not table.is_free(nxt, nt):
                continue                                 # vertex conflict
            if not table.move_ok(cell, nxt, t):
                continue                                 # swap conflict
            state = (nxt, nt)
            if state in seen:
                continue
            seen.add(state)
            came_from[state] = (cell, t)
            f = (nt - start_t) + manhattan(nxt, goal)     # g is just the elapsed time
            heapq.heappush(open_heap, (f, next(tie), state))

    return None


def _reconstruct(came_from: dict[State, State], state: State) -> list[Cell]:
    path = [state[0]]
    while state in came_from:
        state = came_from[state]
        path.append(state[0])
    path.reverse()
    return path
