from fleet.world import Cell


class ReservationTable:
    """Shared booking system: which cells and moves are taken, and when."""

    def __init__(self):
        self.vertices: set[tuple[Cell, int]] = set()          # (cell, t): someone is here at time t
        self.edges: set[tuple[Cell, Cell, int]] = set()       # (a, b, t): someone moves a -> b between t and t+1
        self.parked: dict[Cell, int] = {}                     # cell -> time from which someone sits there forever
        self.last_time: dict[Cell, int] = {}                  # cell -> latest time anyone has booked it
        self.goal_cells: dict[Cell, int] = {}                 # cell -> id of the robot whose goal it is

    def is_free(self, cell: Cell, t: int) -> bool:
        """Can a robot be in this cell at time t?"""
        if (cell, t) in self.vertices:
            return False
        parked_from = self.parked.get(cell)
        return parked_from is None or t < parked_from

    def move_ok(self, a: Cell, b: Cell, t: int) -> bool:
        """Can a robot move a -> b between t and t+1? Blocked if someone moves b -> a at the same time (a swap)."""
        return (b, a, t) not in self.edges

    def can_park(self, cell: Cell, t: int) -> bool:
        """Can a robot arrive here at time t and then stay forever?"""
        return cell not in self.parked and self.last_time.get(cell, -1) < t

    def park(self, cell: Cell, t: int) -> None:
        """Someone occupies this cell from time t onwards."""
        self.parked[cell] = t

    def reserve_path(self, path: list[Cell], start_t: int) -> None:
        """Book every cell and move in a path. path[i] is the cell at time start_t + i."""
        for i, cell in enumerate(path):
            t = start_t + i
            self.vertices.add((cell, t))
            self.last_time[cell] = max(self.last_time.get(cell, -1), t)
            if i > 0 and path[i - 1] != cell:
                self.edges.add((path[i - 1], cell, t - 1))
        # When the path ends, the robot stays at its goal.
        self.park(path[-1], start_t + len(path) - 1)