from src.basic_avoid.basic_avoid_types import *

# warning : grSim gives distances in millimeters at the moment

# Starting source point of the robot
A = Point(0., 0.)

# Destination point to attain
B = Point(2400., 1700.)

# Position of a dummy robot, that we need to avoid
EN_SRC = Point(900, 1400)

robot_radius = 120  # measured using my cursor on grSim
danger_k = 900  # magic number, TBD properly

# See description file for explanations
danger_circle_radius = 2 * robot_radius + danger_k
