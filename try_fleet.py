# try_fleet.py: 3 robots, planned together with prioritized space-time A*
from fleet.world import World
from fleet.tasks import TaskGenerator
from fleet.robot import Robot
from fleet.planning.astar import manhattan
from fleet.planning.prioritized import plan_prioritized

world = World.from_file("maps/small_warehouse.txt")
gen = TaskGenerator(world, rate=0.3, seed=1)

robots = [
    Robot(id=0, pos=(0, 0)),
    Robot(id=1, pos=(3, 9)),
    Robot(id=2, pos=(6, 0)),
]

waiting = []
all_tasks = []
vertex_collisions = 0
swap_collisions = 0
MAX_PRINTS = 10

for t in range(1, 201):
    # 1. New tasks arrive
    new_tasks = gen.step(t)
    waiting += new_tasks
    all_tasks += new_tasks

    # 2. Allocation (greedy)
    for r in robots:
        if r.is_idle() and waiting:
            task = min(waiting, key=lambda task: manhattan(r.pos, task.pickup))
            waiting.remove(task)
            r.assign(task)

    # 3. Planning: if anyone needs a plan, replan the whole fleet together
    if any(r.goal() is not None and not r.path for r in robots):
        paths = plan_prioritized(world, robots, t)
        for r in robots:
            if r.goal() is not None:
                r.set_path(paths.get(r.id, [r.pos]))   # no path found: wait here and retry next step

    # 4. Move everyone
    old = {r.id: r.pos for r in robots}
    for r in robots:
        r.step(t)
    new = {r.id: r.pos for r in robots}

    # 5a. Same-cell collisions
    positions = list(new.values())
    if len(positions) != len(set(positions)):
        vertex_collisions += 1
        if vertex_collisions + swap_collisions <= MAX_PRINTS:
            print(f"t={t} SAME CELL: {new}")

    # 5b. Swap collisions
    for a in robots:
        for b in robots:
            if (a.id < b.id
                    and old[a.id] != old[b.id]
                    and new[a.id] == old[b.id]
                    and new[b.id] == old[a.id]):
                swap_collisions += 1
                if vertex_collisions + swap_collisions <= MAX_PRINTS:
                    print(f"t={t} SWAP: robot {a.id} and robot {b.id} passed through each other")

done = [task for task in all_tasks if task.is_done]
in_progress = [r.task for r in robots if r.task is not None]
print()
print(f"tasks created:    {len(all_tasks)}")
print(f"tasks completed:  {len(done)}")
print(f"still waiting:    {len(waiting)}")
print(f"in progress:      {len(in_progress)}")
print(f"same-cell steps:  {vertex_collisions}")
print(f"swaps:            {swap_collisions}")