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


@ecs.component
class Position:
    def __init__(self, x=None, y=None):
        self.x = x
        self.y = y


def get_position_data():
    """get all curent position data"""
    return np.array([(e.position.x, e.position.y) for e in ecs.World.join(Position, Velocity)])


def get_velocity_data():
    """get all current velocity data"""
    return np.array([(e.velocity.x, e.velocity.y) for e in ecs.World.join(Position, Velocity)])


@ecs.system
def update_positions():
    """update all positions"""
    for e in ecs.World.join(Position, Velocity):
        e.position.x += e.velocity.x
        e.position.y += e.velocity.y


@ecs.system
def chohesion():
    positions = get_position_data()
    centre_direction = positions - np.mean(positions, axis=0)

    for e, d in zip(ecs.World.join(Position, Velocity), centre_direction):
        e.velocity.x -= COHESION_STRENGTH * d[0]
        e.velocity.y -= COHESION_STRENGTH * d[1]

@ecs.system
def separation():
    positions = get_position_data()



@ecs.system
def check_boundary():
    """check for anything outside of boundaries and wrap them"""
    for e in ecs.World.join(Position, Velocity):
        if e.position.x > XMAX:
            e.position.x -= XMAX
        if e.position.x < XMIN:
            e.position.x += XMAX

        if e.position.y > YMAX:
            e.position.y -= YMAX
        if e.position.y < YMIN:
            e.position.y += YMAX



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
