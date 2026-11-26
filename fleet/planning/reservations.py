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

    def can_visit(self, cell: Cell, t: int, dwell: int) -> bool:
        """Can a robot arrive here at time t and stay for `dwell` more steps (then leave)?"""
        return all(self.is_free(cell, t + k) for k in range(dwell + 1))

    def reserve_path(self, path: list[Cell], start_t: int, stay: bool = True, dwell: int = 0) -> None:
        """Book every cell and move in a path. path[i] is the cell at time start_t + i.

        stay=True: the robot stays at the end of the path for good (parking).
        stay=False: the robot stays `dwell` more steps at the end, then leaves (pickup/drop-off).
        """
        for i, cell in enumerate(path):
            t = start_t + i
            self._book(cell, t)
            if i > 0 and path[i - 1] != cell:
                self.edges.add((path[i - 1], cell, t - 1))
        end_t = start_t + len(path) - 1
        if stay:
            self.park(path[-1], end_t)
        else:
            for k in range(1, dwell + 1):
                self._book(path[-1], end_t + k)

    def _book(self, cell: Cell, t: int) -> None:
        self.vertices.add((cell, t))
        self.last_time[cell] = max(self.last_time.get(cell, -1), t)
