import time

import numpy as np

from client import Client

with Client(host='127.0.0.1') as client:
    robot = client.robots["allies"][0]
    print(client.ball)
    print(robot)
    # robot.goto(lambda: (0., 0., 0.), wait=True)
    robot.command((2, 2, 0), client.NO_KICK, None)
