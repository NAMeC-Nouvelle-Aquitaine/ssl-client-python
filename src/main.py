import time

import numpy as np

from client import Client
from rrt import RRT_star, dijkstra, plot
from rrtplanner import RRTStar


with Client(host='127.0.0.1', key='') as client:
    robot = client.robots["allies"][0]
    print(client.ball)
    print(robot)
    # robot.goto(lambda: (-2., 0., 0.), wait=True)

    startpos = np.array((robot.position[0], robot.position[1]))
    endpos = np.array((3.0, 0.0))
    obstacles = []
    for r in client.robots["allies"].values():
        if r != robot:
            obstacles.append((r.position[0], r.position[1]))
    for r in client.robots["enemies"].values():
        if r != robot:
            obstacles.append((r.position[0], r.position[1]))

    n_iter = 500
    radius = 0.3
    stepSize = 1

    from rrtplanner import RRTStar

    og = np.zeros((90, 60))
    print(og.shape)
    t = np.array((len(og)/2.0, len(og[1])/2.0))
    print(t)
    for row in range(len(og)):
        for v in range(len(og[row])):
            pos = np.array((
                row, v
            )) - t
            pos /= 10.
            for obstacle in obstacles:
                n = np.linalg.norm(pos - obstacle)
                if n <= radius:
                    og[row][v] = 1

    startpos *= 10
    endpos *= 10
    startpos += t
    endpos += t

    n = 600
    r_rewire = 10  # large enough for our 400x400 world
    rrts = RRTStar(og, n, r_rewire)

    # xstart = random_point_og(og)
    # xgoal = random_point_og(og)

    T, gv = rrts.plan(startpos, endpos)

    path = rrts.route2gv(T, gv)
    path_pts = rrts.vertices_as_ndarray(T, path)
    print(path_pts)

    from rrtplanner import plot_rrt_lines, plot_path, plot_og, plot_start_goal
    import matplotlib.pyplot as plt

    # create figure and ax.
    fig = plt.figure()
    ax = fig.add_subplot()

    # these functions alter ax in-place.
    plot_og(ax, og)
    plot_start_goal(ax, startpos, endpos)
    plot_rrt_lines(ax, T)
    plot_path(ax, path_pts)

    plt.show()

    for point in path_pts:
        point = (point[1] - t)/10.
        print(point)
        robot.goto((*point, 0), wait=True)
    # while True:
    #     robot.control(0, 0, 0)
    #
    # G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    #
    # if G.success:
    #     path = dijkstra(G)
    #
    #     for p in path:
    #         robot.goto((*p, 0), wait=True)
    #
    # else:
    #     plot(G, obstacles, radius)
