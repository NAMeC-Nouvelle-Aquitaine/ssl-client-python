import time

import numpy as np

from client import Client

with Client(host='127.0.0.1', key='') as client:
    robot = client.robots["blue"][5]
    print(client.ball)
    print(robot)
    while True:
        for i in np.arange(client.ball[1] - 0.3, client.ball[1] + 0.3,0.1):
            robot.goto((-4.4, i, 0),wait=False)
        for i in np.arange(client.ball[1] + 0.3, client.ball[1] - 0.3,-0.1):
            robot.goto((-4.4,i,0),wait=False)
        if client.ball[0] <= -3.4:
            robot.goto((client.ball[0]-0.05,client.ball[1],0))
            robot.dribble(100)
            robot.kick(100)