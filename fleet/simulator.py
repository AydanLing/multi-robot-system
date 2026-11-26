from dataclasses import dataclass
from typing import Callable

from fleet.allocation.parking import send_idle_robots_to_park
from fleet.robot import Robot, State
from fleet.tasks import Task, TaskGenerator
from fleet.world import World, Cell

# A planner takes (world, robots, t) and returns {robot_id: path}.
Planner = Callable[[World, list[Robot], int], dict[int, list[Cell]]]
# An allocator takes (robots, waiting tasks) and assigns tasks in place.
Allocator = Callable[[list[Robot], list[Task]], None]


@dataclass
class RobotSnapshot:
    id: int
    pos: Cell
    state: State
    goal: Cell | None
    path: list[Cell]          # remaining planned cells


@dataclass
class Frame:
    """Everything the visualizer needs to draw one time step."""
    t: int
    robots: list[RobotSnapshot]
    waiting_pickups: list[Cell]
    collisions: list[Cell]    # cells where a collision happened this step
    completed: int


class Simulator:
    def __init__(self, world: World, robots: list[Robot], tasks: TaskGenerator,
                 allocator: Allocator, planner: Planner, parking: bool = True):
        self.world = world
        self.robots = robots
        self.tasks = tasks
        self.allocator = allocator
        self.planner = planner
        self.parking = parking    # send idle robots to parking spots?
        if parking and len(world.parking) < len(robots):
            raise ValueError(f"{len(robots)} robots but only {len(world.parking)} parking spots: "
                             f"every robot needs a spot (add P cells to the map)")
        self.replan_next = True   # replan at the next step (set whenever any robot changes state)
        self.t = 0
        self.waiting: list[Task] = []
        self.all_tasks: list[Task] = []
        self.vertex_collisions = 0
        self.swap_collisions = 0
        self.wait_steps = 0       # robot-steps spent standing still while having somewhere to go
        self.frames: list[Frame] = [self._snapshot([])]

    def step(self) -> Frame:
        self.t += 1
        t = self.t

        # 1. New tasks arrive
        new_tasks = self.tasks.step(t)
        self.waiting += new_tasks
        self.all_tasks += new_tasks

        # 2. Allocate tasks, then send any robot still idle to a parking spot
        self.allocator(self.robots, self.waiting)
        if self.parking:
            send_idle_robots_to_park(self.world, self.robots)

        # 3. Plan: replan the whole fleet if anyone needs a plan, or if anything changed last step
        #    (a robot picked up, dropped off, went idle or parked: earlier plans may be out of date)
        if self.replan_next or any(r.goal() is not None and not r.path for r in self.robots):
            self.replan_next = False
            paths = self.planner(self.world, self.robots, t)
            for r in self.robots:
                if r.goal() is not None:
                    r.set_path(paths.get(r.id, [r.pos]))   # no path: wait and retry

        # 4. Move
        old = {r.id: r.pos for r in self.robots}
        old_states = {r.id: r.state for r in self.robots}
        for r in self.robots:
            r.step(t)
        new = {r.id: r.pos for r in self.robots}
        if any(r.state != old_states[r.id] for r in self.robots):
            self.replan_next = True
        for r in self.robots:
            if r.task is not None and old[r.id] == new[r.id]:
                self.wait_steps += 1

        # 5. Check for collisions
        collisions: list[Cell] = []
        seen: dict[Cell, int] = {}
        for rid, cell in new.items():
            if cell in seen:
                collisions.append(cell)
            seen[cell] = rid
        if collisions:
            self.vertex_collisions += 1
        for a in self.robots:
            for b in self.robots:
                if (a.id < b.id and old[a.id] != old[b.id]
                        and new[a.id] == old[b.id] and new[b.id] == old[a.id]):
                    self.swap_collisions += 1
                    collisions.append(new[a.id])

        frame = self._snapshot(collisions)
        self.frames.append(frame)
        return frame

    def run(self, steps: int) -> None:
        for _ in range(steps):
            self.step()

    @property
    def completed(self) -> int:
        return sum(task.is_done for task in self.all_tasks)

    def summary(self) -> str:
        in_progress = sum(r.task is not None for r in self.robots)
        return (f"tasks created:    {len(self.all_tasks)}\n"
                f"tasks completed:  {self.completed}\n"
                f"still waiting:    {len(self.waiting)}\n"
                f"in progress:      {in_progress}\n"
                f"same-cell steps:  {self.vertex_collisions}\n"
                f"swaps:            {self.swap_collisions}\n"
                f"waiting steps:    {self.wait_steps}")

    def _snapshot(self, collisions: list[Cell]) -> Frame:
        robots = [RobotSnapshot(r.id, r.pos, r.state, r.goal(), list(r.path)) for r in self.robots]
        return Frame(self.t, robots, [task.pickup for task in self.waiting], collisions, self.completed)
