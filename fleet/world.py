from pathlib import Path

Cell = tuple[int,int] # (row, col)

FREE = "."
SHELF = "#"
STATION = "S"

class World:
    def __init__(self, grid:list[str]):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])

        self.stations: list[Cell] = self.find(STATION) # makes the attribute "stations"  that is a list which consists of Cell objects with the Station tag
        self.pickups: list[Cell] = [c for c in self.free_cells() if self._next_to_shelf(c)]

    @classmethod # indicates a method that belongs to the class, rather than individual instances of the class
    def from_file(cls, path: str | Path) -> "World" : # "path: str | Path" uses the type hint "str |" to suggest either a plain string or a path object from python's pathlib
        lines = Path(path).read_text().splitlines()
        grid = [line.strip() for line in lines if line.strip()] # skips blank lines
        return cls(grid)

    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width # return 0 if the rows or columns exceed the bounds of the World
    
    def is_free(self, cell: Cell) -> bool:
        # True if a robot can stand here (inside the map and not a shelf)
        if not self.in_bounds(cell):
            return False
        r, c = cell
        return self.grid[r][c] != SHELF

    def neighbours(self, cell: Cell) -> list[Cell]:
        # Free cells reachable in one move: up, down, left, right 
        r, c = cell
        candidates = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]
        return [n for n in candidates if self.is_free(n)]

    def find(self, symbol: str) -> list[Cell]:
        # All cells containing a given symbol

        return [
            (r, c)
            for r in range(self.height)
                for c in range(self.width)
                    if self.grid[r][c] == symbol
        ]

    def free_cells(self) -> list[Cell]:
        return [
            (r, c)
            for r in range(self.height)
                for c in range(self.width)
                    if self.is_free((r, c))
        ]

    def _next_to_shelf(self, cell: Cell) -> bool:
        # A pickup spot is a free cell beside a shelf (where a robot would pick)
        r, c = cell
        if self.grid[r][c] == STATION:
            return False
        for n in [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]:
            r, c = cell
            if self.in_bounds(n) and self.grid[n[0]][n[1]] == SHELF:
                return True
        return False

    def render(self, robots: dict[int, Cell] | None = None) -> str:
        # Text picture of the map, with robot ids drawn on top
        rows = [list(row) for row in self.grid]
        for robot_id, (r, c) in (robots or {}).items():
            rows[r][c] = str(robot_id % 10) # single character per cell
        return "\n".join("".join(row) for row in rows)