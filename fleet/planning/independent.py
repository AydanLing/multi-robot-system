from fleet.planning.astar import astar
from fleet.robot import Robot
from fleet.world import World, Cell


def plan_independent(world: World, robots: list[Robot], t: int) -> dict[int, list[Cell]]:
    """Plan each robot alone with plain A*, ignoring every other robot. (The version that crashes.)"""
    paths = {}
    for r in robots:
        if r.goal() is not None:
            path = astar(world, r.pos, r.goal())
            if path is not None:
                paths[r.id] = path
    return paths
