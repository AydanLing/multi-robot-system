from fleet.planning.astar import manhattan
from fleet.robot import Robot, State
from fleet.world import World


def send_idle_robots_to_park(world: World, robots: list[Robot]) -> None:
    """Every robot that just went idle drives to the closest parking spot nobody has claimed.

    A spot is claimed if a robot is parked on it or heading to it. If every spot is claimed,
    the robot stays where it is.
    """
    taken = {r.park_spot for r in robots if r.state in (State.TO_PARKING, State.PARKED)}
    for r in robots:
        if r.state != State.IDLE:          # has a task, or is already parking/parked
            continue
        free = [spot for spot in world.parking if spot not in taken]
        if not free:
            continue
        spot = min(free, key=lambda spot: manhattan(r.pos, spot))
        r.send_to_park(spot)
        taken.add(spot)                    # so the next idle robot doesn't pick the same one
