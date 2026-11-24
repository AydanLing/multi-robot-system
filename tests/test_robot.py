from fleet.robot import Robot, State
from fleet.tasks import Task


def make_task():
    return Task(id=0, pickup=(0, 2), dropoff=(0, 4), created_at=0)


def test_new_robot_is_idle():
    assert Robot(id=0, pos=(0, 0)).is_idle()


def test_assign_sets_goal_to_pickup():
    r = Robot(id=0, pos=(0, 0))
    r.assign(make_task())
    assert r.state == State.TO_PICKUP
    assert r.goal() == (0, 2)


def test_full_task_cycle():
    r = Robot(id=0, pos=(0, 0))
    task = make_task()
    r.assign(task)
    r.set_path([(0, 0), (0, 1), (0, 2)])
    r.step(1)
    r.step(2)                           # reaches pickup
    assert r.state == State.TO_DROPOFF
    assert task.picked_up_at == 2
    r.set_path([(0, 2), (0, 3), (0, 4)])
    r.step(3)
    r.step(4)                           # reaches dropoff
    assert r.is_idle()
    assert task.completed_at == 4


def test_robots_do_not_share_paths():
    a, b = Robot(id=0, pos=(0, 0)), Robot(id=1, pos=(1, 0))
    a.path.append((0, 1))
    assert b.path == []