from client import Client
from client import ClientRobot
from basic_avoid.basic_avoid_consts import *
from basic_avoid.basic_avoid_utils import *
import numpy as np

global client

ally: ClientRobot
enemy: ClientRobot
crab_minion: ClientRobot


def declare_robots():
    """
    Initializes global variables
    """
    print("Setting up variables...")
    # blue0 is the ally
    global ally
    ally = client.robots['blue'][0]

    # consider blue1 as enemy
    global enemy
    enemy = client.robots['blue'][1]

    # this guy is just a dummy
    global crab_minion
    crab_minion = client.robots['blue'][2]


def place_robots(ally_pos: Point, dst_pos: Point, enemy_pos: Point):
    """
    Setting up the placement of the robots to showcase the strategy
    """
    print("Placing robots...")
    ally.goto((ally_pos.x, ally_pos.y, 0.))
    enemy.goto((enemy_pos.x, enemy_pos.y, 0.))
    crab_minion.goto((dst_pos.x, dst_pos.y, 0.))


def visualize_circle(robot: ClientRobot, radius: float):
    """
    Moves the enemy robot around the edges of its danger circle
    in grSim. This is pure visualization, but also very slow.
    """
    print("Visualizing danger circle..")
    x_rob, y_rob = robot.position[0], robot.position[1]
    # Place on 4 edges of circle
    for deg in range(0, 360, 90):
        x = x_rob + (radius * np.sin(np.deg2rad(deg)))
        y = y_rob + (radius * np.cos(np.deg2rad(deg)))
        enemy.goto((x, y, float(angle_towards(Point(x, y), Point(x_rob, y_rob)))))

    # Back to spawn
    robot.goto((x_rob, y_rob, 0.))


def ally_goto_and_avoid(robot: ClientRobot, dst: Point, avoid: ClientRobot):
    """
    Send a goto command to the 'ally' robot by avoiding the enemy robot, which will compute
    extra waypoints to go to if necessary

    Works for one robot, but should be not too painful to scale up to n robots
    """
    print("Starting 'avoid-like' goto...")
    # Save the source position of the robot
    src = Point(robot.position[0], robot.position[1])

    # Get the danger circle of the enemy
    dgr_circle = Circle(
        Point(avoid.position[0], avoid.position[1]),
        danger_circle_radius
    )

    _, is_circle_crossed = compute_intersections(circle=dgr_circle, line=(src, dst))
    if not is_circle_crossed:
        # No avoiding necessary, just go to the position
        ally.goto(dst)
    else:
        waypoint = compute_waypoint(circle=dgr_circle, line=(src, dst))
        ally.goto((waypoint.x, waypoint.y, 0.))
        print("     - Waypoint attained")
        ally.goto((dst.x, dst.y, 0.))
        print("     - Destination attained")


def run(given_client: Client):
    global client
    client = given_client

    declare_robots()
    for src, dst, avoid_pos in xyt_datasets:
        place_robots(ally_pos=src, dst_pos=dst, enemy_pos=avoid_pos)
        visualize_circle(robot=enemy, radius=danger_circle_radius)
        ally_goto_and_avoid(robot=ally, dst=dst, avoid=enemy)

    print("finished !")
    exit(0)

