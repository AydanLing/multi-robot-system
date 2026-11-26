"""Does sending idle robots to parking pockets help? Same robots, same tasks, several seeds.

Run from the repo root:  python -m experiments.compare_parking
"""
from statistics import mean

from fleet.allocation.greedy import allocate_greedy
from fleet.planning.prioritized import plan_prioritized
from fleet.robot import Robot
from fleet.simulator import Simulator
from fleet.tasks import TaskGenerator
from fleet.world import World

# (name, map, parking on?)
SETUPS = [
    ("original map",          "maps/small_warehouse.txt",   False),
    ("pockets, no parking",   "maps/warehouse_parking.txt", False),
    ("pockets + parking",     "maps/warehouse_parking.txt", True),
]
START_CELLS = [(0, 0), (3, 9), (6, 0), (0, 9), (3, 0)]
N_ROBOTS = 3
STEPS = 300
SEEDS = range(30)


STALL = 80        # a run counts as stuck if robots have tasks but nothing was delivered for this many steps


def run_once(path: str, parking: bool, rate: float, seed: int) -> tuple[Simulator, bool]:
    world = World.from_file(path)
    robots = [Robot(id=i, pos=START_CELLS[i]) for i in range(N_ROBOTS)]
    sim = Simulator(world, robots, TaskGenerator(world, rate=rate, seed=seed),
                    allocator=allocate_greedy, planner=plan_prioritized, parking=parking)
    last_delivery_t, delivered = 0, 0
    for _ in range(STEPS):
        sim.step()
        if sim.completed != delivered:
            delivered, last_delivery_t = sim.completed, sim.t
    stuck = STEPS - last_delivery_t > STALL and any(r.task is not None for r in robots)
    return sim, stuck


def main():
    for rate in (0.3, 0.1):
        print(f"\ntask rate {rate}: {N_ROBOTS} robots, {STEPS} steps, {len(SEEDS)} seeds")
        print(f"{'setup':<22} {'created':>8} {'delivered':>10} {'waiting steps':>14} {'stuck runs':>11} {'collisions':>11}")
        for name, path, parking in SETUPS:
            results = [run_once(path, parking, rate, seed) for seed in SEEDS]
            sims = [sim for sim, _ in results]
            stuck = sum(st for _, st in results)
            created = mean(len(s.all_tasks) for s in sims)
            delivered = mean(s.completed for s in sims)
            waiting = mean(s.wait_steps for s in sims)
            collisions = sum(s.vertex_collisions + s.swap_collisions for s in sims)
            print(f"{name:<22} {created:>8.1f} {delivered:>10.1f} {waiting:>14.1f} {f'{stuck}/{len(SEEDS)}':>11} {collisions:>11}")


if __name__ == "__main__":
    main()
