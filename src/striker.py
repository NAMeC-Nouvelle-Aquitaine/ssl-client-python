from builtins import classmethod

from client import ClientRobot, Client
import numpy as np


class Striker:
    # Represents this robot instance
    __robot: ClientRobot

    def __init__(self, robot: ClientRobot):
        self.__robot = robot

    @classmethod
    def compute_preshoot_pos(self, target: np.array, ball: np.array):
        # Vector from target (where ball should go) towards ball
        target_to_ball: np.array = ball - target

        # Normalize the vector
        target_to_ball = target_to_ball / np.linalg.norm(target_to_ball)  # TODO : can divide with 0  if target == ball
        # Change the vector's origin to start from the ball
        position_pre_shoot: np.array = ball + target_to_ball
        position_pre_shoot_angle = np.arctan2(ball[1] - position_pre_shoot[1], ball[0] - position_pre_shoot[0])
        return position_pre_shoot[0], position_pre_shoot[1], position_pre_shoot_angle
    def run(self, target: np.array, client: Client):
        ball: np.array = client.ball
        
        # Compute pos to go to before ball
        preshoot_pos = Striker.compute_preshoot_pos(target, ball)

        # Dynamically go towards pre-shoot position at least once
        last_ball_pos: np.array = ball
        while not np.isclose(self.__robot.position, preshoot_pos[:2]).all():
            self.__robot.goto(preshoot_pos, wait=False)

            # If ball moved, do it again
            if not np.isclose(ball, last_ball_pos).all():
                preshoot_pos = Striker.compute_preshoot_pos(target, ball)

        print("[PRESHOOT POS] Placed & prepared")

        # Then, go towards ball until we get it under us
        while not self.__robot.infrared:
            self.__robot.goto(
                (ball[0], ball[1], preshoot_pos[2]),
                wait=False
            )
            self.__robot.dribble(200)
        print("[SHOOT] Placed & prepared")

        # Kick that ball towards target
        robot_to_target = np.arctan2(
            target[1] - self.__robot.position[1],
            target[0] - self.__robot.position[0],
        )
        self.__robot.goto(
            (ball[0], ball[1], robot_to_target)
        )

        self.__robot.kick(1.)
        # Weird command to stop the robot's motors totally
        for _ in range(15):
            self.__robot.control(0., 0., 0.)
