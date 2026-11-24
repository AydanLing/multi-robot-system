from fleet.world import World

w = World.from_file("maps/small_warehouse.txt")
print(w.render({0: (0, 0), 1: (7, 4)}))
print("stations:", w.stations)
print("pickups:", len(w.pickups))
print("neighbours of (0,0):", w.neighbours((0, 0)))