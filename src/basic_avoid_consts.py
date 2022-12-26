from collections import namedtuple

# Custom types
Point = namedtuple('Point', ['x', 'y'])
Circle = namedtuple('Circle', ['center', 'r'])

# warning : grSim gives distances in millimeters at the moment
A = Point(0., 0.)
B = Point(2400., 1700.)

EN_SRC = Point(900, 1400)

robot_radius = 120  # measured using my cursor on grSim
danger_k = 600  # magic number, TBD properly

danger_circle_radius = 2 * robot_radius + danger_k
