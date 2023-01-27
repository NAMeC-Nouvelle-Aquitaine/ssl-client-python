import time
import numpy as np

from client import Client

with Client(host='127.0.0.1', key='') as client:
    finished = False
    while not finished:
        numero_of_robot = int(input("Please enter the numero of your robot"))
        if numero_of_robot < 0 or numero_of_robot > 6:
            print("Number notvalid")
        else:
            finished = True;
    robot = client.robots["allies"][numero_of_robot]
    while True:
        for i in np.arange(client.ball[1] - 0.3, client.ball[1] + 0.3,0.1):
            robot.goto((-1.5, i, 0),wait=False)
        for i in np.arange(client.ball[1] + 0.3, client.ball[1] - 0.3,-0.1):
            robot.goto((-1.5,i,0),wait=False)
        if client.ball[0] <= -3.4:
            robot.goto((client.ball[0]-0.05,client.ball[1],0))
            robot.dribble(100)
            robot.kick(0.5)