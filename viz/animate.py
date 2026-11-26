"""Draw simulator frames with matplotlib: live in a window, or saved as a GIF."""
import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Polygon, Rectangle

from fleet.robot import State
from fleet.simulator import Frame
from fleet.world import World, SHELF, STATION, PARKING, WALL

FLOOR = "#f3f2ee"
SHELF_COLOR = "#4a4a4a"
STATION_COLOR = "#cfe8d5"
PARKING_COLOR = "#dfe6f2"
WALL_COLOR = "#2b2b2b"
PICKUP_COLOR = "#e8952f"
COLLISION_COLOR = "#d62728"
ROBOT_COLORS = ["#1f77b4", "#9467bd", "#2a9d8f", "#e377c2", "#8c564b",
                "#bcbd22", "#393b79", "#7f7f7f", "#17becf", "#6b8e23"]


EXPLOSION_STEPS = 2.0      # how many time steps an explosion stays on screen


def _starburst(x, y, radius, spikes=12, inner=0.45, twist=0.0):
    """Points of a jagged star centred on (x, y)."""
    pts = []
    for i in range(spikes * 2):
        r = radius if i % 2 == 0 else radius * inner
        a = math.pi * i / spikes + twist
        pts.append((x + r * math.cos(a), y + r * math.sin(a)))
    return pts


def _draw_explosion(ax, cell, age):
    """Explosion at `cell`, `age` time steps after the crash: grows fast, then fades out."""
    x, y = _xy(cell)
    grow = min(age / 0.5, 1.0)                     # full size after half a step
    fade = max(0.0, 1.0 - age / EXPLOSION_STEPS)
    size = 0.35 + 0.55 * grow
    layers = [  # (scale, colour, twist): red outside, yellow inside, white-hot core
        (1.00, "#d62728", 0.0),
        (0.72, "#ff7f0e", 0.26),
        (0.45, "#ffd23f", 0.0),
        (0.20, "#fffbe6", 0.26),
    ]
    artists = []
    for scale, color, twist in layers:
        star = Polygon(_starburst(x, y, size * scale, twist=twist + age), closed=True,
                       facecolor=color, edgecolor="none", alpha=fade, zorder=9, clip_on=False)
        ax.add_patch(star)
        artists.append(star)
    if age < 1.2:
        artists.append(ax.text(x, y - size - 0.15, "CRASH!", ha="center", va="bottom",
                               fontsize=9, fontweight="bold", color="#d62728", alpha=fade, zorder=10, clip_on=False))
    return artists


def _xy(cell):
    """(row, col) -> (x, y) for plotting: x is the column, y is the row."""
    r, c = cell
    return c, r


def _draw_map(ax, world: World):
    for r in range(world.height):
        for c in range(world.width):
            ch = world.grid[r][c]
            color = {SHELF: SHELF_COLOR, STATION: STATION_COLOR, PARKING: PARKING_COLOR, WALL: WALL_COLOR}.get(ch, FLOOR)
            ax.add_patch(Rectangle((c - 0.5, r - 0.5), 1, 1, facecolor=color, edgecolor="white", linewidth=1))
            if ch == STATION:
                ax.text(c, r, "S", ha="center", va="center", fontsize=9, color="#2f6b3c", fontweight="bold")
            if ch == PARKING:
                ax.text(c, r, "P", ha="center", va="center", fontsize=9, color="#3b5998", fontweight="bold")
    ax.set_xlim(-0.5, world.width - 0.5)
    ax.set_ylim(world.height - 0.5, -0.5)      # row 0 at the top, like the map file
    ax.set_aspect("equal")
    ax.axis("off")


class _Panel:
    """One side of the animation: a map plus everything that moves on it."""

    def __init__(self, ax, world: World, frames: list[Frame], title: str):
        self.ax, self.world, self.frames, self.title = ax, world, frames, title
        _draw_map(ax, world)
        self.dynamic = []                       # artists redrawn every animation frame
        # Running collision count at each step, for the title
        self.collision_count = []
        total = 0
        for f in frames:
            total += bool(f.collisions)
            self.collision_count.append(total)

    def draw(self, step: int, alpha: float):
        """Draw time step `step`, with robots moved `alpha` (0 to 1) of the way to step + 1."""
        for artist in self.dynamic:
            artist.remove()
        self.dynamic = []
        ax = self.ax
        cur = self.frames[step]
        nxt = self.frames[min(step + 1, len(self.frames) - 1)]
        positions_next = {s.id: s.pos for s in nxt.robots}

        # Waiting tasks: orange diamonds at their pickup cells
        for cell in cur.waiting_pickups:
            x, y = _xy(cell)
            self.dynamic += ax.plot(x, y, marker="D", color=PICKUP_COLOR, markersize=6, zorder=2)

        for s in cur.robots:
            color = ROBOT_COLORS[s.id % len(ROBOT_COLORS)]
            # Planned path: faint line
            if s.path:
                xs, ys = zip(*[_xy(c) for c in [s.pos] + s.path])
                self.dynamic += ax.plot(xs, ys, color=color, alpha=0.35, linewidth=2.5, zorder=3)
            # Goal: outlined square
            if s.goal is not None:
                gx, gy = _xy(s.goal)
                sq = Rectangle((gx - 0.4, gy - 0.4), 0.8, 0.8, fill=False, edgecolor=color, linewidth=2, zorder=3)
                ax.add_patch(sq)
                self.dynamic.append(sq)
            # Robot: circle, smoothly moving towards its next position
            x0, y0 = _xy(s.pos)
            x1, y1 = _xy(positions_next.get(s.id, s.pos))
            x, y = x0 + (x1 - x0) * alpha, y0 + (y1 - y0) * alpha
            body = Circle((x, y), 0.36, facecolor=color, edgecolor="white", linewidth=1.5, zorder=5)
            ax.add_patch(body)
            self.dynamic.append(body)
            if s.state == State.TO_DROPOFF:     # carrying a package: small square on the robot
                box = Rectangle((x - 0.13, y - 0.13), 0.26, 0.26, facecolor="#f7d08a", edgecolor="#7a5a1e", zorder=6)
                ax.add_patch(box)
                self.dynamic.append(box)
            else:
                self.dynamic.append(ax.text(x, y, str(s.id), ha="center", va="center",
                                            fontsize=8, color="white", fontweight="bold", zorder=6))

        # Explosions: each crash happens halfway through a move, then stays on screen for a bit
        now = step + alpha
        first = max(1, int(now - EXPLOSION_STEPS))
        for k in range(first, min(step + 2, len(self.frames))):
            age = now - (k - 0.5)
            if 0 <= age < EXPLOSION_STEPS:
                for cell in set(self.frames[k].collisions):
                    self.dynamic += _draw_explosion(ax, cell, age)

        shown = nxt if alpha > 0.5 else cur
        collisions = self.collision_count[min(step + (alpha > 0.5), len(self.frames) - 1)]
        ax.set_title(f"{self.title}\nt={shown.t}   delivered={shown.completed}   collisions={collisions}",
                     fontsize=10, color=COLLISION_COLOR if collisions else "#222222")


def animate(world: World, runs: list[tuple[str, list[Frame]]], out: str | None = None,
            substeps: int = 4, fps: int = 14):
    """Animate one or more simulator runs side by side.

    runs: list of (title, frames). out: GIF path to save, or None to show a live window.
    substeps: drawing frames per time step (more = smoother motion).
    """
    n_steps = min(len(frames) for _, frames in runs) - 1
    fig, axes = plt.subplots(1, len(runs), figsize=(4.2 * len(runs), 4.2 * world.height / world.width + 0.6))
    if len(runs) == 1:
        axes = [axes]
    panels = [_Panel(ax, world, frames, title) for ax, (title, frames) in zip(axes, runs)]
    fig.tight_layout()

    def update(i):
        step, sub = divmod(i, substeps)
        for p in panels:
            p.draw(step, sub / substeps)
        return []

    anim = FuncAnimation(fig, update, frames=n_steps * substeps, interval=1000 / fps, blit=False)
    if out:
        anim.save(out, writer=PillowWriter(fps=fps))
        plt.close(fig)
        print(f"saved {out}")
    else:
        plt.show()
    return anim
