from fleet.world import Cell

class ReservationTable:
    # a shared booking system: which cells and moves are taken, and when
    def __init__(self):
        self.vertices: set[tuple[Cell, int]] = set() # (cell, t): someone is here at time t
        self.edges: set[tuple[Cell, Cell, int]] = set() # (a, b, t): someone moves a -> b between t and t+1
        self.parked: dict[Cell, int] = {} # cell -> time from which a robot is parked there after finishing a task (forever)
        self.last_time: dict[Cell, int] = {} # cell -> latest time anyone has booked it

    def is_free(self, cell: Cell, t: int) -> bool:
        # can a robot be in this cell a time t?
        if (cell, t) in self.vertices: # if the cell is currently occupied as a part of their path
            return False
        parked_from = self.parked.get(cell) # checks if anyone is parked there after their task is complete
        return parked_from is None or t < parked_from 

    def move_ok(self, a: Cell, b: Cell, t: int) -> bool:
        # checks for swap collisions
        return (b, a, t) not in self.edges # checks if this movement is not already queued

    def can_park(self, cell: Cell, t: int) -> bool:
        # can a robot park here indefinitely
        return cell not in self.parked and self.last_time.get(cell, -1) < t

    def park(self, cell: Cell, t: int) -> None:
        # updates the reservation table to show that someone is parked at this cell
        self.parked[cell] = t

    def reserve_path(self, path: list[Cell], start_t: int) -> None:
        # book every cell and move in a robot's path. path[i] is the cell at time start_t + i
        for i, cell in enumerate(path):
            t = start_t + i
            self.vertices.add((cell, t))
            self.last_time[cell] = max(self.last_time.get(cell, -1), t)
            if i > 0 and path[i-1] != cell:
                self.edges.add((path[i-1], cell, t-1))
        # when the path ends, the robot stays at its goal
        self.park(path[-1], start_t + len(path)-1)

    