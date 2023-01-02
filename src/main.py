import time

import numpy as np

from client import Client, ClientRobot

team = 'blue'
rob_id = 0

# In millimeters, or meters ?
# Will be automatically adapted using scale variable
# Change only if the robot doesn't go correctly straight
# but you shouldn't have to
pos_tol = 2.5


def angle_towards(src: tuple[float, float], dst: tuple[float, float]) -> float:
    """
    Returns the angle in radian from one point towards another one point
    """
    return np.arctan2(
        dst[1] - src[1],
        dst[0] - src[0]
    )


def stop_robot(rob: ClientRobot):
    for _ in range(10):
        rob.control(0., 0., 0.)


def get_field_scale(rob: ClientRobot) -> int:
    _ = input("Have you put the desired robot on the circle around the center of the field ?")
    return 1000 if (abs(rob.position[0]) > 6.5 or abs(rob.position[1] > 4.5)) else 1


def slow_goto(rob: ClientRobot, pos: tuple[float, float], tol: float):
    # orienting towards destination point
    theta = angle_towards(rob.position, pos)
    rob.goto((rob.position[0], rob.position[1], theta))

    # roomba towards point, slowly
    while not np.allclose(rob.position, pos, atol=tol):
        rob.control(0.5, 0, 0)
    stop_robot(rob)
    rob.goto((pos[0], pos[1], theta))
    print(f"Attained {pos}")


def you_make_my_head_go_round_right_round(rob: ClientRobot, tol: float):
    slow_goto(rob, (0, 0), tol)
    for _ in range(10):
        rob.control(0, 0, 2.5)
    stop_robot(rob)
    for _ in range(10):
        rob.control(0, 0, -2.5)
    stop_robot(rob)
    slow_goto(rob, (0, 0), tol)


def run_around_field_celebration(rob: ClientRobot, scale: int, tolerance: float):
    """
    Makes a given robot go around the field

    Note : testing this on grSim made the robots deviate using the .control() method
           a bit weird that this happened.
           If this happens IRL please change the 'tolerance' parameter to something higher
    """

    top_mid = (0., 4.5*scale)
    top_left = (-6*scale, 4.5*scale)
    bottom_left = (-6*scale, -4.5*scale)
    bottom_right = (6*scale, -4.5*scale)
    top_right = (6*scale, 4.5*scale)

    # navigating
    slow_goto(rob, top_mid, tolerance)
    slow_goto(rob, top_left, tolerance)
    slow_goto(rob, bottom_left, tolerance)
    slow_goto(rob, bottom_right, tolerance)
    slow_goto(rob, top_right, tolerance)
    slow_goto(rob, (0., 0.), tolerance)


# Main program !
with Client(host='127.0.0.1', key='') as client:
    robot = client.robots[team][rob_id]

    # Scale determination
    s = get_field_scale(robot)
    print(f"Scale = {s}")

    # Rotating test, undefined speed while rotating
    you_make_my_head_go_round_right_round(robot, tol=pos_tol*s/10)

    # Movement test, go around field SLOWLY
    run_around_field_celebration(robot, s, tolerance=pos_tol*s/10)
