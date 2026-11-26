from fleet.world import World
from fleet.planning.reservations import ReservationTable
from fleet.planning.space_time_astar import space_time_astar
from fleet.planning.prioritized import plan_prioritized
from fleet.robot import Robot
from fleet.tasks import Task

CORRIDOR = ["....."]          # one row: robots can't go around each other


def test_free_world_is_shortest_path():
    w = World(CORRIDOR)
    path = space_time_astar(w, (0, 0), (0, 4), 0, ReservationTable())
    assert path == [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)]


def test_waits_for_booked_cell():
    w = World(CORRIDOR)
    table = ReservationTable()
    table.reserve_path([(0, 2), (0, 2), (0, 3)], 0)   # someone sits on (0,2) until t=1, then parks on (0,3)
    path = space_time_astar(w, (0, 0), (0, 2), 0, table)
    assert path is not None
    assert path[-1] == (0, 2)
    for t, cell in enumerate(path):
        assert table.is_free(cell, t) or t == 0


def test_head_on_corridor_has_no_solution():
    # Two robots facing each other in a 1-wide corridor can never pass.
    w = World(CORRIDOR)
    table = ReservationTable()
    table.reserve_path([(0, 4), (0, 3), (0, 2), (0, 1), (0, 0)], 0)
    assert space_time_astar(w, (0, 0), (0, 4), 0, table) is None


def test_swap_is_blocked():
    w = World(["..", ".."])
    table = ReservationTable()
    table.reserve_path([(0, 1), (0, 0)], 0)            # moves right-to-left at t=0
    path = space_time_astar(w, (0, 0), (0, 1), 0, table)
    # Moving (0,0) -> (0,1) at t=0 would be a swap, so the path must take longer than 1 step.
    assert path is not None and len(path) > 2


def _no_collisions(paths):
    horizon = max(len(p) for p in paths.values()) + 1
    at = lambda p, t: p[min(t, len(p) - 1)]            # after the end, a robot stays at its goal
    ids = list(paths)
    for t in range(horizon):
        cells = [at(paths[i], t) for i in ids]
        assert len(cells) == len(set(cells)), f"same cell at t={t}"
        for a in ids:
            for b in ids:
                if a < b and t > 0:
                    pa0, pa1 = at(paths[a], t - 1), at(paths[a], t)
                    pb0, pb1 = at(paths[b], t - 1), at(paths[b], t)
                    assert not (pa1 == pb0 and pb1 == pa0 and pa0 != pb0), f"swap at t={t}"


def test_prioritized_crossing_robots_do_not_collide():
    w = World.from_file("maps/small_warehouse.txt")
    robots = [Robot(id=0, pos=(3, 0)), Robot(id=1, pos=(3, 9)), Robot(id=2, pos=(0, 4))]
    goals = [(3, 9), (3, 0), (6, 4)]                     # 0 and 1 drive head-on along row 3
    for r, g in zip(robots, goals):
        r.assign(Task(id=r.id, pickup=g, dropoff=(7, 0), created_at=0))
    paths = plan_prioritized(w, robots, 0)
    assert set(paths) == {0, 1, 2}
    for r, g in zip(robots, goals):
        assert paths[r.id][-1] == g
    _no_collisions(paths)


def test_robot_on_someones_goal_moves_aside():
    # The frozen situation from the parking experiment:
    # robot 2 sits on station (7,0) and needs (2,0); robot 1 sits on (2,0) and needs (7,0).
    from fleet.robot import State
    w = World.from_file("maps/small_warehouse.txt")
    r0 = Robot(id=0, pos=(3, 5))
    r0.assign(Task(id=0, pickup=(3, 5), dropoff=(7, 0), created_at=0))
    r0.state = State.TO_DROPOFF
    r1 = Robot(id=1, pos=(2, 0))
    r1.assign(Task(id=1, pickup=(2, 0), dropoff=(7, 0), created_at=0))
    r1.state = State.TO_DROPOFF
    r2 = Robot(id=2, pos=(7, 0))
    r2.assign(Task(id=2, pickup=(2, 0), dropoff=(7, 9), created_at=0))
    paths = plan_prioritized(w, [r0, r1, r2], 0)
    assert paths, "everyone stuck: the old deadlock is back"
    moving = [rid for rid, p in paths.items() if len(set(p)) > 1]
    assert moving, "nobody actually moves"
    _no_collisions(paths)
