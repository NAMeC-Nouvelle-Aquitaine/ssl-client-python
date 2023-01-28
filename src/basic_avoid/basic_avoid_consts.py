# warning : grSim gives distances in millimeters at the moment

robot_radius = 0.06  # measured using my cursor on grSim
danger_k = 0.08  # magic number, TBD properly

# See description file for explanations
danger_circle_radius = 2 * robot_radius + danger_k

# Used in compute_waypoint() to space the waypoint away from the danger circle
# pretty random atm, gotta fix it properly
AVOID_DIST_FACTOR_MAX = 0.6
