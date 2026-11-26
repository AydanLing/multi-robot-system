from fleet.planning.astar import manhattan
from fleet.robot import Robot
from fleet.tasks import Task


def allocate_greedy(robots: list[Robot], waiting: list[Task]) -> None:
    """Each idle robot, in list order, takes the waiting task with the closest pickup.

    Changes robots and the waiting list in place.
    """
    for r in robots:
        if r.is_idle() and waiting:
            task = min(waiting, key=lambda task: manhattan(r.pos, task.pickup))
            waiting.remove(task)
            r.assign(task)
