import heapq
from itertools import count

from fleet.planning.astar import manhattan
from fleet.planning.reservations import ReservationTable
from fleet.world import World, Cell

State = tuple[Cell, int] # cell, time

def space_time_astar(
        world: World,
        start: Cell,
        goal: Cell,
        start_t: int,
        table: ReservationTable,
        max_steps: int | None = None,
) -> list[Cell] | None:
    # this planner finds the shortest path, while avoiding everything booked in the reservation table
    # Returns none if no plan is found withing max_steps

    if not world.is_free(start) or not world.is_free(goal):
        return None # the start or goal positions are invalid
    if goal in table.parked:
        return None # a robot is parked in the goal, give it up
    if max_steps is None:
        max_steps = 2*world.width*world.height

    tie = count()
    start_state: State = (start, start_t)
    open_heap = [(manhattan(start, goal), next(tie), start_state)]
    came_from: dict[State, State] = {}
    seen: set[State] = {start_state}

    while open_heap:
        _, _, (cell, t) = heapq.heappop(open_heap)

        if cell == goal and table.can_park(goal, t):
            return _reconstruct(came_from, (cell, t))

        if t-start_t >= max_steps:
            continue

        for nxt in world.neighbours(cell) + [cell]:
            nt = t+1
            if not table.is_free(nxt, nt):
                continue
            if not table.move_ok(cell, nxt, t):
                continue
            state = (nxt, nt)
            if state in seen:
                continue
            seen.add(state)
            came_from[state] = (cell, t)
            f = (nt-start_t) + manhattan(nxt, goal)

            heapq.heappush(open_heap, (f, next(tie), state))

    return None

def _reconstruct(came_from: dict[State, State], state: State) -> list[Cell]:
    path = [state[0]]
    while state in came_from:
        state = came_from[state]
        path.append(state[0])
    path.reverse()
    return path

