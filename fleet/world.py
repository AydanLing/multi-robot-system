from pathlib import Path

Cell = tuple[int, int]  # (row, col)

FREE = "."
SHELF = "#"
STATION = "S"
PARKING = "P"
WALL = "X"      # blocked like a shelf, but not a place to pick items from
BLOCKED = {SHELF, WALL}


class World:
    def __init__(self, grid: list[str]):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])
        for i, row in enumerate(grid):
            if len(row) != self.width:
                raise ValueError(f"Row {i} has length {len(row)}, expected {self.width}")
        self.stations: list[Cell] = self.find(STATION)
        self.parking: list[Cell] = self.find(PARKING)
        self.pickups: list[Cell] = [c for c in self.free_cells() if self._next_to_shelf(c)]

    @classmethod
    def from_file(cls, path: str | Path) -> "World":
        lines = Path(path).read_text().splitlines()
        grid = [line.strip() for line in lines if line.strip()]
        return cls(grid)

    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, cell: Cell) -> bool:
        if not self.in_bounds(cell):
            return False
        r, c = cell
        return self.grid[r][c] not in BLOCKED

    def neighbours(self, cell: Cell) -> list[Cell]:
        r, c = cell
        candidates = [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]
        return [n for n in candidates if self.is_free(n)]

    def find(self, symbol: str) -> list[Cell]:
        return [(r, c) for r in range(self.height) for c in range(self.width) if self.grid[r][c] == symbol]

    def free_cells(self) -> list[Cell]:
        return [(r, c) for r in range(self.height) for c in range(self.width) if self.is_free((r, c))]

    def _next_to_shelf(self, cell: Cell) -> bool:
        r, c = cell
        if self.grid[r][c] in (STATION, PARKING):
            return False
        for n in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
            if self.in_bounds(n) and self.grid[n[0]][n[1]] == SHELF:
                return True
        return False
