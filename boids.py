"""
"""
import numpy as np

from matplotlib import animation
from matplotlib import pyplot as plt


import execs as ecs


XMIN, XMAX = 0.0, 250.0
YMIN, YMAX = 0.0, 250.0
BOUNDARY_LAYER = 5.0
BOUNDARY_FACTOR = 1.1

MAX_SPEED = 2.0

COHESION_STRENGTH = 0.005
SEPARATION_STRENGTH = 0.05
ALIGNMENT_STRENGTH = 0.05

VISUAL_RANGE = 75.0
VISUAL_ANGLE = np.pi / 4.0
SEPARATION_RANGE = 5.0


@ecs.component
class Velocity:
    def __init__(self, x=None, y=None):
        self.x = x
        self.y = y

    def bearing(self):
        """bearing of velocity"""
        return np.arctan2(self.y, self.x)

    def __call__(self):
        return np.array((self.x, self.y))


@ecs.component
class Position:
    def __init__(self, x=None, y=None):
        self.x = x
        self.y = y

    def __call__(self):
        return np.array((self.x, self.y))


@ecs.component
class Close:
    def __init__(self, position_data=None, velocity_data=None):
        self.position_data = position_data
        self.velocity_data = velocity_data

    def __bool__(self):
        return len(self.position_data) > 0


def get_position_data():
    """get all curent position data"""
    return np.array([e.position() for e in ecs.World.join(Position, Velocity)])


def get_velocity_data():
    """get all current velocity data"""
    return np.array([e.velocity() for e in ecs.World.join(Position, Velocity)])


def get_close(boid, all_positions, all_velocities):
    """return positions and velocities of close boids"""

    position = boid.position()
    other_mask = (all_positions == position).sum(axis=1) < 2

    other_positions = all_positions[other_mask, :]
    other_velocities = all_velocities[other_mask, :]

    distance = np.linalg.norm(other_positions - position, axis=1)
    close = distance < VISUAL_RANGE

    if not close.any():
        return [], []

    other_positions = other_positions[close, :]
    other_velocities = other_velocities[close, :]

    angle = np.arctan2(other_positions[:, 1] - boid.position.y,
                       other_positions[:, 0] - boid.position.x)
    seen = np.abs(angle - boid.velocity.bearing()) < VISUAL_ANGLE

    if not seen.any():
        return [], []

    return other_positions[seen, :], other_velocities[seen, :]


def create_random_flock(n):
    for _ in range(n):
        boid = world.create()
        boid.attach(Position(x=np.random.uniform(XMIN, XMAX),
                             y=np.random.uniform(YMIN, YMAX)))
        boid.attach(Velocity(x=np.random.uniform(-0.1, 0.1),
                             y=np.random.uniform(-0.1, 0.1)))


def get_quiver_data():
    entities = ecs.World.join(Position, Velocity)

    positions = []
    directions = []
    for e in entities:
        positions.append((e.position.x, e.position.y))
        directions.append((e.velocity.x, e.velocity.y))

    return np.array(positions), np.array(directions)


if __name__ == "__main__":
    world = ecs.World()

    create_random_flock(5)

    fig = plt.figure(tight_layout=True)
    ax = fig.add_subplot(111)
    ax.set(xlim=(XMIN, XMAX), ylim=(YMIN, YMAX))

    positions, directions = get_quiver_data()
    plot = ax.quiver(positions[:, 0], positions[:, 1], directions[:, 0], directions[:, 1],
        pivot="middle")

    def animate(frame):
        world.run_systems()

        positions, directions = get_position_data(), get_velocity_data()
        plot.set_offsets(positions)
        plot.set_UVC(directions[:, 0], directions[:, 1])

    anim = animation.FuncAnimation(fig, animate, frames=50, interval=3)

    plt.show()
