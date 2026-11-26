from fleet.planning.astar import manhattan
from fleet.planning.reservations import ReservationTable
from fleet.planning.space_time_astar import space_time_astar
from fleet.robot import Robot
from fleet.world import World, Cell

YIELD_CANDIDATES = 10     # how many nearby cells a yielding robot tries before giving up


def plan_prioritized(world: World, robots: list[Robot], t: int) -> dict[int, list[Cell]]:
    """Plan every robot that has a goal, one at a time, in list order (earlier = higher priority).

    Idle robots stay where they are. If a robot can't find a path:
      - if it's standing on another robot's goal, it YIELDS: it plans a short path to a nearby
        unused cell instead, and gets to plan first so the others can plan around it;
      - otherwise it's marked stuck and waits in place.
    Then everyone replans. Returns {robot_id: path} for robots that got a path.
    """
    goal_cells = {r.goal(): r.id for r in robots if r.goal() is not None}
    stuck: set[int] = set()
    yielding: set[int] = set()

    while True:
        table = ReservationTable()
        table.goal_cells = goal_cells

        # Robots with nowhere to go (idle or stuck) occupy their cell from now on.
        for r in robots:
            if r.goal() is None or r.id in stuck:
                table.park(r.pos, t)

        paths: dict[int, list[Cell]] = {}
        failed: Robot | None = None

        # Yielding robots go first: get off other robots' goals.
        for r in robots:
            if r.id in yielding:
                path = _plan_yield(world, r, t, table)
                if path is None:
                    failed = r
                    break
                table.reserve_path(path, t)
                paths[r.id] = path

        # Then everyone else plans to their real goal, in priority order.
        if failed is None:
            for r in robots:
                if r.goal() is None or r.id in stuck or r.id in yielding:
                    continue
                path = space_time_astar(world, r.pos, r.goal(), t, table)
                if path is None:
                    failed = r
                    break
                table.reserve_path(path, t)
                paths[r.id] = path

        if failed is None:
            return paths

        # Decide what the failed robot does on the next attempt.
        blocking = goal_cells.get(failed.pos, failed.id) != failed.id
        if failed.id in yielding or not blocking:
            yielding.discard(failed.id)
            stuck.add(failed.id)        # can't get out of the way either: wait in place
        else:
            yielding.add(failed.id)     # standing on someone's goal: move aside


def _plan_yield(world: World, robot: Robot, t: int, table: ReservationTable) -> list[Cell] | None:
    """Path to the closest cell that isn't anyone's goal and that the robot can stay in."""
    candidates = [c for c in world.free_cells()
                  if c != robot.pos and c not in table.goal_cells and c not in world.stations]
    candidates.sort(key=lambda c: manhattan(robot.pos, c))
    for cell in candidates[:YIELD_CANDIDATES]:
        path = space_time_astar(world, robot.pos, cell, t, table)
        if path is not None:
            return path
    return None
