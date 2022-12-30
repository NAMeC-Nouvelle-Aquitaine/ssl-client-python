from src.basic_avoid.basic_avoid_types import *

# warning : grSim gives distances in millimeters at the moment

# Point naming :
#   S_i : Source point of the ally robot
#   D_i : Destination point that the ally robot must reach
#   E_i : Enemy position to avoid
S_1 = Point(0., 0.)
D_1 = Point(2400., 1700.)
E_1 = Point(900, 1400)

# Array of src, dst, enemy position coordinates used by the main program
xyt_datasets = [
    (S_1, D_1, E_1)
]

robot_radius = 120  # measured using my cursor on grSim
danger_k = 900  # magic number, TBD properly

# See description file for explanations
danger_circle_radius = 2 * robot_radius + danger_k
