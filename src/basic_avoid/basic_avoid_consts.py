from basic_avoid.basic_avoid_types import *

# warning : grSim gives distances in millimeters at the moment

# Point naming :
#   S_i : Source point of the ally robot
#   D_i : Destination point that the ally robot must reach
#   E_i : Enemy position to avoid
S_1 = Point(0., 0.)
D_1 = Point(2.4, 1.7)
E_1 = Point(0.9, 1.4)

# Array of src, dst, enemy position coordinates used by the main program
xyt_datasets = [
    (S_1, D_1, E_1)
]

robot_radius = 0.12  # measured using my cursor on grSim
danger_k = 0.9  # magic number, TBD properly

# See description file for explanations
danger_circle_radius = 2 * robot_radius + danger_k
