import time

import numpy as np

from client import Client

with Client(host='127.0.0.1', key='') as client:
    robot = client.robots["blue"][0]
    print(client.ball)
    print(robot)
    # robot.goto(lambda: (0., 0., 0.), wait=True)

    p0 = robot.position
    p1 = np.array([2., 2.])
    p2 = np.array([0., 0.])

    p0_to_p1 = p1 - p0
    p1_to_p2 = p2 - p1

    for lerp in np.linspace(0, 1, 100):
        P1 = p0 + p0_to_p1 * lerp
        P2 = p1 + p1_to_p2 * lerp

        P1_to_P2 = P2 - P1

        p = P1 + P1_to_P2 * lerp
        print(p)
        robot.goto((*p, 0))
