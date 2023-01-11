import time

import numpy as np

from client import Client, ClientRobot


def stop_robot(rob: ClientRobot):
    for _ in range(10):
        rob.control(0., 0., 0.)


def slow_goto(rob: ClientRobot, pos: tuple[float, float], tol: float):
    angle = np.arctan2(
        pos[1] - rob.position[1],
        pos[0] - rob.position[0]
    )
    rob.goto((rob.position[0], rob.position[1], angle))

    # roomba towards point, slowly
    while not np.allclose(rob.position, pos, atol=tol):
        rob.control(0.5, 0, 0)
    stop_robot(rob)
    rob.goto((*pos, 0.))
    print(f"Attained {pos}")


def run_around_field_celebration(rob: ClientRobot, tolerance: float, rectangle_edges: tuple[float, float]):
    """
    Makes a given robot go around the field

    Note : testing this on grSim made the robots deviate using the .control() method
           a bit weird that this happened.
    """
    # navigating
    slow_goto(rob, (rectangle_edges[0], rectangle_edges[1]), tolerance)
    slow_goto(rob, (-rectangle_edges[0], rectangle_edges[1]), tolerance)
    slow_goto(rob, (-rectangle_edges[0], -rectangle_edges[1]), tolerance)
    slow_goto(rob, (rectangle_edges[0], -rectangle_edges[1]), tolerance)
    slow_goto(rob, (0., 0.), tolerance)


# Main program !
with Client(host='127.0.0.1', key='') as client:
    print("Running..")
    # Constants
    team: str = 'blue'
    rob_id: int = 0
    robot: ClientRobot = client.robots[team][rob_id]
    pos_tol: float = 3.
    field_edges = [0.9, 0.9]

    # Movement test, go around field SLOWLY
    run_around_field_celebration(robot, tolerance=pos_tol, rectangle_edges=field_edges)
