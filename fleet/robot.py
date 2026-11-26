from dataclasses import dataclass, field
from enum import Enum

from fleet.tasks import Task
from fleet.world import Cell


class State(Enum):
    IDLE = "idle"
    TO_PICKUP = "to_pickup"
    TO_DROPOFF = "to_dropoff"
    TO_PARKING = "to_parking"
    PARKED = "parked"


@dataclass
class Robot:
    id: int
    pos: Cell
    state: State = State.IDLE
    task: Task | None = None
    path: list[Cell] = field(default_factory=list)
    park_spot: Cell | None = None          # parking spot it's heading to or parked on

    def is_idle(self) -> bool:
        """True if the robot has no task (idle, heading to park, or parked): free to take a task."""
        return self.task is None

    def goal(self) -> Cell | None:
        if self.state == State.TO_PICKUP:
            return self.task.pickup
        if self.state == State.TO_DROPOFF:
            return self.task.dropoff
        if self.state == State.TO_PARKING:
            return self.park_spot
        return None

    def assign(self, task: Task) -> None:
        self.task = task
        task.assigned_to = self.id
        self.state = State.TO_PICKUP
        self.park_spot = None           # a task always beats parking
        self.path = []

    def send_to_park(self, spot: Cell) -> None:
        self.park_spot = spot
        self.state = State.TO_PARKING
        self.path = []

    def set_path(self, path: list[Cell]) -> None:
        if path and path[0] == self.pos:
            path = path[1:]
        self.path = list(path)

    def step(self, t: int) -> None:
        if self.path:
            self.pos = self.path.pop(0)
        self._check_arrival(t)

    def _check_arrival(self, t: int) -> None:
        if self.state == State.TO_PICKUP and self.pos == self.task.pickup:
            self.task.picked_up_at = t
            self.state = State.TO_DROPOFF
            self.path = []
        elif self.state == State.TO_DROPOFF and self.pos == self.task.dropoff:
            self.task.completed_at = t
            self.task = None
            self.state = State.IDLE
            self.path = []
        elif self.state == State.TO_PARKING and self.pos == self.park_spot:
            self.state = State.PARKED
            self.path = []
