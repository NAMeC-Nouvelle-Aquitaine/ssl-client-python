from src.client import Client
from src.client import ClientRobot
from src.basic_avoid.basic_avoid_consts import *
from src.basic_avoid.basic_avoid_utils import *
from typing import Callable
import numpy as np

global client

ally: ClientRobot
enemy: ClientRobot
crab_minion: ClientRobot


def declare_robots():
    # blue0 is the ally
    global ally
    ally = client.robots['blue'][0]

    # consider blue1 as enemy
    global enemy
    enemy = client.robots['blue'][1]

    # this guy is just a dummy
    global crab_minion
    crab_minion = client.robots['blue'][2]


def place_robots():
    ally.goto((A.x, A.y, 0.))
    enemy.goto((EN_SRC.x, EN_SRC.y, 0.))
    crab_minion.goto((B.x, B.y, 0.))


def visualize_circle(center: Point, radius: float):
    # Place on 4 edges of circle
    for deg in range(0, 360, 90):
        x = center.x + (radius * np.sin(np.deg2rad(deg)))
        y = center.y + (radius * np.cos(np.deg2rad(deg)))
        enemy.goto((x, y, float(np.deg2rad(deg))))

    # Back to spawn
    enemy.goto((EN_SRC.x, EN_SRC.y, 0.))


def ally_goto_and_avoid(robot:ClientRobot, dst: Point, avoid: ClientRobot):
    # Save the source position of the robot
    src = Point(robot.position[0], robot.position[1])

    # Get the danger circle of the enemy
    dgr_circle = Circle(
        Point(avoid.position[0], avoid.position[1]),
        danger_circle_radius
    )

    intersection_points = compute_intersections(circle=dgr_circle, line=(src, dst))
    # TODO: continue with logic


def run(given_client: Client):
    global client
    client = given_client

    declare_robots()
    place_robots()
    visualize_circle(center=EN_SRC, radius=danger_circle_radius)
    ally_goto_and_avoid(robot=ally, dst=B, avoid=enemy)

    print("finished !")
    exit(0)

