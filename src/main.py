import time

import numpy as np

from client import Client

def lerp(p1, p2, t):
    p1_to_p2 = p2 - p1
    return p1 + p1_to_p2 * t

with Client(host='127.0.0.1', key='') as client:
    robot = client.robots["blue"][0]
    print(client.ball)
    print(robot)
    # robot.goto(lambda: (0., 0., 0.), wait=True)

    p0 = robot.position
    p1 = np.array([2., 2.])
    p2 = np.array([0., 0.])

    ANIMATION_TIME = 3.0 # in seconds

    t0 = time.time()
    get_t = lambda: (time.time() - t0) / ANIMATION_TIME

    while time.time() < t0 + ANIMATION_TIME:
        P1 = lerp(p0, p1, get_t())
        P2 = lerp(p1, p2, get_t())

        robot.goto((*lerp(P1, P2, get_t()), 0.0), wait=False)
    robot.control(0, 0, 0)
