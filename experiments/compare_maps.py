"""Does adding stations reduce queuing? Same robots, same task rate, several seeds.

Run from the repo root:  python -m experiments.compare_maps
"""
from statistics import mean

from fleet.allocation.greedy import allocate_greedy
from fleet.planning.prioritized import plan_prioritized
from fleet.robot import Robot
from fleet.simulator import Simulator
from fleet.tasks import TaskGenerator
from fleet.world import World

MAPS = {
    "2 stations": "maps/small_warehouse.txt",
    "4 stations": "maps/warehouse_4_stations.txt",
}
START_CELLS = [(0, 0), (3, 9), (6, 0), (0, 9), (3, 0)]
N_ROBOTS = 3
STEPS = 300
RATE = 0.3
SEEDS = range(5)


def run_once(world: World, seed: int) -> Simulator:
    robots = [Robot(id=i, pos=START_CELLS[i]) for i in range(N_ROBOTS)]
    sim = Simulator(world, robots, TaskGenerator(world, rate=RATE, seed=seed),
                    allocator=allocate_greedy, planner=plan_prioritized)
    sim.run(STEPS)
    return sim


def main():
    print(f"{N_ROBOTS} robots, {STEPS} steps, task rate {RATE}, {len(SEEDS)} seeds\n")
    print(f"{'map':<12} {'delivered':>10} {'waiting steps':>14} {'collisions':>11}")
    for name, path in MAPS.items():
        world = World.from_file(path)
        sims = [run_once(world, seed) for seed in SEEDS]
        delivered = mean(s.completed for s in sims)
        waiting = mean(s.wait_steps for s in sims)
        collisions = sum(s.vertex_collisions + s.swap_collisions for s in sims)
        print(f"{name:<12} {delivered:>10.1f} {waiting:>14.1f} {collisions:>11}")


if __name__ == "__main__":
    main()