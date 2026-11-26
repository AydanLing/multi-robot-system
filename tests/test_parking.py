from fleet.allocation.parking import send_idle_robots_to_park
from fleet.robot import Robot, State
from fleet.tasks import Task
from fleet.world import World

GRID = ["....",
        "S...",
        "XPXP"]


def test_parking_cells_are_not_pickups():
    w = World(GRID)
    assert w.parking == [(2, 1), (2, 3)]
    assert (2, 1) not in w.pickups and (2, 3) not in w.pickups
    assert not w.is_free((2, 0))                       # X is a wall


def test_idle_robots_pick_different_closest_spots():
    w = World(GRID)
    a, b = Robot(id=0, pos=(1, 1)), Robot(id=1, pos=(1, 0))
    send_idle_robots_to_park(w, [a, b])
    assert a.goal() == (2, 1)                          # closest to a
    assert b.goal() == (2, 3)                          # (2,1) already claimed


def test_task_beats_parking():
    r = Robot(id=0, pos=(0, 0))
    r.send_to_park((2, 1))
    assert r.is_idle()                                 # still free to take a task
    r.assign(Task(id=0, pickup=(0, 3), dropoff=(1, 0), created_at=0))
    assert r.state == State.TO_PICKUP and r.park_spot is None


def test_arriving_at_spot_parks():
    r = Robot(id=0, pos=(1, 1))
    r.send_to_park((2, 1))
    r.set_path([(1, 1), (2, 1)])
    r.step(1)
    assert r.state == State.PARKED and r.goal() is None