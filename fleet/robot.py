from dataclasses import dataclass, field
from enum import Enum

from fleet.tasks import Task
from fleet.world import Cell

class State(Enum):
    IDLE = "idle"
    TO_PICKUP = "to_pickup"
    TO_DROPOFF = "to_dropoff"

@dataclass
class Robot:
    id: int
    pos: Cell
    state: State = State.IDLE
    task: Task | None = None
    path: list[Cell] = field(default_factory = list)

    def is_idle(self) -> bool:
        return self.state == State.IDLE

    def goal(self) -> Cell | None:
        # Where the roboti s currently heading
        if self.state == State.TO_PICKUP:
            return self.task.pickup
        if self.state == State.TO_DROPOFF:
            return self.task.dropoff
        return None

    def assign(self, task: Task) -> None:
        self.task = task
        task.assigned_to = self.id
        self.state = State.TO_PICKUP
        self.path = []

    def set_path(self, path: list[Cell]) -> None:
        # give the robot a planned path. path[0] should be its current position
        if path and path[0] == self.pos: # it is already standing in the path
            path = path[1:] # drop the cell it's already standing on
        self.path = list(path)

    def step(self, t: int) -> None:
        # advance one time step by moving one cell then checking for arrival
        if self.path:
            self.pos = self.path.pop(0)
        self._check_arrival(t)

    def _check_arrival(self, t: int) -> None:
        if self.state == State.TO_PICKUP and self.pos == self.task.pickup: # it arrived at the pickup location
            self.task.picked_up_at = t
            self.state = State.TO_DROPOFF
            self.path = [] # new goal, need new plan
        elif self.state == State.TO_DROPOFF and self.pos == self.task.dropoff:  # it arrived at the dropoff location
            self.task.completed_at = t
            self.task = None
            self.state = State.IDLE
            self.path = []
            