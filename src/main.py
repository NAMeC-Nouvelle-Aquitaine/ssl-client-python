import time

import numpy as np

from client import Client
from rrt import RRT_star, dijkstra, plot

with Client(host='127.0.0.1', key='') as client:
    robot = client.robots["allies"][0]
    print(client.ball)
    print(robot)
    # robot.goto(lambda: (-2., 0., 0.), wait=True)

    startpos = (robot.position[0], robot.position[1])
    endpos = (3.0, 0.0)
    obstacles = []
    for r in client.robots["allies"].values():
        if r != robot:
            obstacles.append((r.position[0], r.position[1]))
    for r in client.robots["enemies"].values():
        if r != robot:
            obstacles.append((r.position[0], r.position[1]))

    n_iter = 100
    radius = 0.2
    stepSize = 0.3

    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)

    if G.success:
        path = dijkstra(G)
        print(path)

        plot(G, obstacles, radius, path)
        for p in path:
            robot.goto((*p, 0), wait=True)

    else:
        plot(G, obstacles, radius)
