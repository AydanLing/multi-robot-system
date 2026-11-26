import pytest

from fleet.allocation.greedy import allocate_greedy
from fleet.planning.prioritized import plan_prioritized
from fleet.robot import Robot
from fleet.simulator import Simulator
from fleet.tasks import TaskGenerator
from fleet.world import World


def make_sim(map_path, n_robots, parking, rate=0.3, seed=0):
    world = World.from_file(map_path)
    starts = [(0, 0), (3, 9), (6, 0), (0, 9), (3, 0), (6, 9)]
    robots = [Robot(id=i, pos=starts[i]) for i in range(n_robots)]
    return Simulator(world, robots, TaskGenerator(world, rate=rate, seed=seed),
                     allocator=allocate_greedy, planner=plan_prioritized, parking=parking)


def test_not_enough_parking_spots_is_an_error():
    with pytest.raises(ValueError):
        make_sim("maps/warehouse_parking.txt", n_robots=5, parking=True)   # 4 spots, 5 robots


def test_no_collisions_with_parking():
    sim = make_sim("maps/warehouse_parking.txt", n_robots=4, parking=True)
    sim.run(300)
    assert sim.vertex_collisions == 0 and sim.swap_collisions == 0


def test_safety_net_when_robots_stay_on_stations():
    # Without parking, robots that finish just sit on the station, breaking the planner's
    # "it leaves right after" assumption. Replanning on every state change must still prevent crashes.
    for seed in range(5):
        sim = make_sim("maps/small_warehouse.txt", n_robots=3, parking=False, rate=0.1, seed=seed)
        sim.run(300)
        assert sim.vertex_collisions == 0 and sim.swap_collisions == 0, f"seed {seed}"