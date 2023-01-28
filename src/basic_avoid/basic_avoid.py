import numpy as np

from .basic_avoid_consts import danger_circle_radius
from .basic_avoid_types import Circle
from .basic_avoid_utils import angle_towards, compute_intersections, compute_waypoint, danger_circle, distance, \
    point_symmetry, closest_to_dst, circle_gen_eq
from client import ClientRobot, Client

global ally, enemy, crab_minion


def declare_robots(client: Client):
    global ally, enemy, crab_minion
    ally = client.robots["allies"][0]
    enemy = client.robots["allies"][1]
    crab_minion = client.robots["allies"][2]


def goto_avoid(robot: ClientRobot, dst: np.array, avoid_list: list[ClientRobot]):
    """
    Send a goto command to the 'ally' robot by avoiding the enemy robot, which will compute
    extra waypoints to go to if necessary
    """

    # Save the source position of the robot
    src = robot.position
    waypoints = []
    dgr_circles_to_avoid = [danger_circle(rob) for rob in avoid_list]

    # Compute waypoints to avoid each robot in the avoid list
    for enn in avoid_list:
        dgr_circle = danger_circle(enn)
        _, is_circle_crossed = compute_intersections(circle=dgr_circle, line=(src, dst))

        if is_circle_crossed:
            print("collision detected")
            if np.linalg.norm(dst - src) > np.linalg.norm(dst - enn.position):
                # print(f"enn pos : {enn.position}")
                # print(f"waypoints : {waypoints}")
                print(f"guy to avoid : {enn.number}")
                waypoints.append(compute_waypoint(dgr_circle, (src, dst), dgr_circles_to_avoid))

    # If there are waypoints to go to
    if len(waypoints) > 0:
        # Grab the waypoint closest to the robot, and closest to the destination
        wp = closest_to_dst(waypoints, src)
        print(f"going to waypoint {wp}")
        robot.goto((wp[0], wp[1], 0.), wait=True)
    else:
        # Otherwise you go to the destination
        print("straight to dst")
        robot.goto((dst[0], dst[1], 0.), wait=True)


def visualize_circle(robot: ClientRobot, circle: Circle):
    """
    Moves the enemy robot around the edges of its danger circle
    in grSim. This is pure visualization, but also very slow.
    """
    x_rob, y_rob = robot.position
    r: float = circle.r
    edges = [
        (x_rob + r, y_rob,     0.),
        (x_rob,     y_rob + r, 0.),
        (x_rob - r, y_rob,     0.),
        (x_rob,     y_rob - r, 0.)
    ]

    print("Visualizing danger circle..")
    for pos in edges:
        robot.goto(pos)


def step(client: Client):
    declare_robots(client)

    # Visualization
    # client.robots["allies"][3].goto((0., 0., 0.))
    # r = client.robots["allies"][3]
    # visualize_circle(r, danger_circle(r))

    # roots, inter = compute_intersections(
    #     danger_circle(client.robots["enemies"][3]),
    #     (client.robots["allies"][0].position, client.robots["allies"][2].position)
    # )
    # print(inter)

    avoid_list = [client.robots["enemies"][i] for i in (0, 1, 2, 3, 4)]
    goto_avoid(robot=ally, dst=crab_minion.position, avoid_list=avoid_list)

    # Symmetry test
    # client.robots["allies"][1].goto(
    #     (*point_symmetry(client.robots["allies"][5].position, client.robots["allies"][3].position), 0.)
    # )

