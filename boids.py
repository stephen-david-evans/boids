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
    """create a random flock of boids uniformly across parameter space"""
    for _ in range(n):
        boid = world.create()
        boid.attach(Position(x=np.random.uniform(XMIN, XMAX),
                             y=np.random.uniform(YMIN, YMAX)))
        boid.attach(Velocity(x=np.random.uniform(-MAX_SPEED, MAX_SPEED),
                             y=np.random.uniform(-MAX_SPEED, MAX_SPEED)))
        boid.attach(Close(position_data=None, velocity_data=None))


@ecs.system
def find_close():
    """find close boids and update local subsets of data"""
    all_positions, all_velocities = get_position_data(), get_velocity_data()
    for boid in ecs.World.join(Position, Velocity):
        close_postions, close_velocities = get_close(boid, all_positions, all_velocities)

        boid.close.position_data = close_postions
        boid.close.velocity_data = close_velocities


@ecs.system
def limit_speed():
    """limit top speed of boids"""
    for e in ecs.World.gather(Velocity):
        speed = np.linalg.norm((e.velocity.x, e.velocity.y))
        if speed > MAX_SPEED:
            e.velocity.x = MAX_SPEED * (e.velocity.x / speed)
            e.velocity.y = MAX_SPEED * (e.velocity.y / speed)


@ecs.system
def check_boundary():
    """apply periodic boundary conditions"""
    for e in ecs.World.join(Position, Velocity):
        if e.position.x < XMIN + BOUNDARY_LAYER:
            e.velocity.x += BOUNDARY_FACTOR

        if e.position.x > XMAX - BOUNDARY_LAYER:
            e.velocity.x -= BOUNDARY_FACTOR

        if e.position.y < YMIN + BOUNDARY_LAYER:
            e.velocity.y += BOUNDARY_FACTOR

        if e.position.y > YMAX - BOUNDARY_LAYER:
            e.velocity.y -= BOUNDARY_FACTOR


@ecs.system
def chohesion():
    """each boid flys towards local centre of mass"""
    for boid in ecs.World.join(Position, Velocity, Close):
        if not boid.close:
            continue

        centre = np.mean(boid.close.position_data, axis=0)

        boid.velocity.x += COHESION_STRENGTH * (centre[0] - boid.position.x)
        boid.velocity.y += COHESION_STRENGTH * (centre[1] - boid.position.y)


@ecs.system
def separation():
    """each boid will try and avoid other boids"""
    for boid in ecs.World.join(Position, Velocity, Close):
        if not boid.close:
            continue

        position = boid.position()

        distance = np.linalg.norm(boid.close.position_data - position, axis=1)
        move = np.sum(position - boid.close.position_data[distance < SEPARATION_RANGE], axis=0)

        boid.velocity.x += SEPARATION_STRENGTH * move[0]
        boid.velocity.y += SEPARATION_STRENGTH * move[1]


@ecs.system
def alignment():
    """each boid will try and match velocity to close boids"""
    for boid in ecs.World.join(Position, Velocity, Close):
        if not boid.close:
            continue

        centre = np.mean(boid.close.velocity_data, axis=0)

        boid.velocity.x += ALIGNMENT_STRENGTH * (centre[0] - boid.velocity.x)
        boid.velocity.y += ALIGNMENT_STRENGTH * (centre[1] - boid.velocity.y)


@ecs.system
def update_positions():
    """update all positions"""
    for e in ecs.World.join(Position, Velocity):
        e.position.x += e.velocity.x
        e.position.y += e.velocity.y


if __name__ == "__main__":
    world = ecs.World()

    create_random_flock(100)

    fig = plt.figure(tight_layout=True)
    ax = fig.add_subplot(111)
    ax.set(xlim=(XMIN, XMAX), ylim=(YMIN, YMAX))

    positions, directions = get_position_data(), get_velocity_data()
    next_positions = positions + directions

    body, = ax.plot(positions[:, 0], positions[:, 1], c="k", markersize=6, marker="o", ls="none")
    head, = ax.plot(next_positions[:, 0], next_positions[:, 1], c="r", markersize=3, marker="o", ls="none")

    def animate(frame, body, head):
        world.run_systems()

        positions, directions = get_position_data(), get_velocity_data()
        next_positions = positions + directions

        body.set_data(positions[:, 0], positions[:, 1])
        head.set_data(next_positions[:, 0], next_positions[:, 1])

    anim = animation.FuncAnimation(fig, animate, interval=0, fargs=(body, head))

    plt.show()
