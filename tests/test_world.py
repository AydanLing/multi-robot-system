from fleet.world import World

GRID = [
    "...",
    ".#.",
    "S..",
]

def test_shelf_is_blocked():
    w = World(GRID)
    assert not w.is_free((1,1))

def test_out_of_bounds_is_blocked():
    w = World(GRID)
    assert not w.is_free((-1, 0))
    assert not w.is_free((0, 3))

def test_neighbours_avoid_shelf():
    w = World(GRID)
    assert (1, 1) not in w.neighbours((0, 1))

def test_stations_found():
    w = World(GRID)
    assert w.stations == [(2, 0)]
    