from fleet.planning.reservations import ReservationTable
from fleet.planning.space_time_astar import space_time_astar
from fleet.robot import Robot
from fleet.world import World, Cell

def plan_prioritized(world: World, robots: list[Robot], t: int) -> dict[int, list[Cell]]:
    # distributes the tasks one at a time in order of the Robot list
    # returns {robot_id: path} for rboots that got a path

    stuck: set[int] = set()

    while True:
        table = ReservationTable()

        # Robots with nowhere to go occupy their cell from now on
        for r in robots:
            if r.goal() is None or r.id in stuck:
                table.park(r.pos, t)

        paths: dict[int, list[Cell]] = {}
        failed: int | None = None

        for r in robots: 
            if r.goal() is None or r.id in stuck:
                continue
            path = space_time_astar(world, r.pos, r.goal(), t, table)
            if path is None:
                failed = r.id
                break
            table.reserve_path(path, t)
            paths[r.id] = path

        if failed is None:
            return paths
        stuck.add(failed)

        