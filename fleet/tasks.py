import random
from dataclasses import dataclass

from fleet.world import World, Cell

@dataclass
class Task:
    id: int
    pickup: Cell
    dropoff: Cell
    created_at: int # the time stamp that the task was created at
    assigned_to: int | None = None # the robot id, once assigned
    picked_up_at: int | None = None
    completed_at: int | None = None

    @property
    def is_done(self) -> bool:
        return self.completed_at is not None # this just gets the status

class TaskGenerator:
    # creates random tasks over time

    def __init__(self, world: World, rate: float, seed: int = 0):
        self.world = world
        self.rate = rate
        self.rng = random.Random(seed)
        self.next_id = 0

    def step(self, t: int) -> list[Task]:
        # call once per time step. returns new tasks
        if self.rng.random() < self.rate:
            return [self._make_task(t)]
        return []

    def _make_task(self, t: int) -> Task:
        task = Task(
            id=self.next_id,
            pickup=self.rng.choice(self.world.pickups),
            dropoff=self.rng.choice(self.world.stations),
            created_at=t,
        )    
        self.next_id += 1
        return task

    