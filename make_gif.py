"""Watch the fleet, or save a side-by-side GIF.

    python make_gif.py            # live window: coordinated fleet
    python make_gif.py --compare  # live window: plain A* (crashing) vs space-time A*
    python make_gif.py --compare --save results/compare.gif
"""
import argparse

from fleet.allocation.greedy import allocate_greedy
from fleet.planning.independent import plan_independent
from fleet.planning.prioritized import plan_prioritized
from fleet.robot import Robot
from fleet.simulator import Simulator
from fleet.tasks import TaskGenerator
from fleet.world import World
from viz.animate import animate

START_CELLS = [(0, 0), (3, 9), (6, 0), (0, 9), (3, 0)]


def run(planner, args, world):
    robots = [Robot(id=i, pos=START_CELLS[i]) for i in range(args.robots)]
    sim = Simulator(world, robots, TaskGenerator(world, rate=args.rate, seed=args.seed),
                    allocator=allocate_greedy, planner=planner, parking=args.parking)
    sim.run(args.steps)
    return sim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", default="maps/small_warehouse.txt")
    parser.add_argument("--robots", type=int, default=3)
    parser.add_argument("--steps", type=int, default=60)
    parser.add_argument("--rate", type=float, default=0.3)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--parking", action="store_true", help="idle robots drive to P cells")
    parser.add_argument("--compare", action="store_true", help="show plain A* next to space-time A*")
    parser.add_argument("--save", help="save a GIF to this path instead of opening a window")
    args = parser.parse_args()

    world = World.from_file(args.map)
    runs = []
    if args.compare:
        naive = run(plan_independent, args, world)
        runs.append(("Plain A* (robots ignore each other)", naive.frames))
    smart = run(plan_prioritized, args, world)
    runs.append(("Space-time A* + reservations", smart.frames))

    animate(world, runs, out=args.save)


if __name__ == "__main__":
    main()
