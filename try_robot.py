from fleet.world import World
from fleet.tasks import TaskGenerator
from fleet.robot import Robot
from fleet.planning.astar import astar

world = World.from_file("maps/small_warehouse.txt")
gen = TaskGenerator(world, rate=1.0, seed=42)
robot = Robot(id=0, pos=(0,0))

task = gen.step(t=0)[0]
robot.assign(task)
print("task:", task)

for t in range(1, 40):
    if robot.goal() is not None and not robot.path:
        robot.set_path(astar(world, robot.pos, robot.goal()))
    robot.step(t)
    print(f"t={t} state={robot.state.value} pos={robot.pos}")
    if robot.is_idle():
        print("task done:", task)
        break