from builtins import classmethod

from client import ClientRobot, Client
import numpy as np

from field_observer import FieldObserver


class Striker:
    # Represents this robot instance
    __robot: ClientRobot

    def __init__(self, robot: ClientRobot):
        self.__robot = robot

    @classmethod
    def compute_preshoot_pos(cls, target: np.array, ball: np.array):
        """
        Compute the position in (x, y) to where the robot should position itself
        to be just in front of the ball, aiming towards a certain position.
        """
        # Vector from target (where the ball should go) towards ball
        target_to_ball: np.array = ball - target

        # Normalize the vector
        norm: float = np.linalg.norm(target_to_ball)
        norm = 1 if norm == 0 else norm
        target_to_ball = target_to_ball / norm

        # Change the vector's origin to start from the ball
        position_pre_shoot: np.array = ball + target_to_ball
        position_pre_shoot_angle = np.arctan2(ball[1] - position_pre_shoot[1], ball[0] - position_pre_shoot[0])
        return position_pre_shoot[0], position_pre_shoot[1], position_pre_shoot_angle

    @classmethod
    def determine_shoot_target_to_goal(cls, goal_posts: np.ndarray[np.ndarray], enemy_robs: list[ClientRobot]):
        """
        Computes a point towards which the ball should go

        Parameters :
            goal_posts | Coordinates of left and right enemy goal posts
            enemy_robs | List of ClientRobot objects representing the enemy positions

        Returns :
            A 2-dimensional np array, the coordinates of the target to aim towards
        """
        enemy_gk: ClientRobot = FieldObserver.guess_goal_keeper(enemy_robs, goal_posts)
        wanted_post: np.array = FieldObserver.get_furthest_post_of_gk(enemy_gk, goal_posts)

        # Apply a small offset to the wanted post
        # # TODO: another offset to put the target inside the goal posts
        gk_posts_mid: np.array = (goal_posts[0] + goal_posts[1]) / 2
        vec_post_to_mid: np.array = gk_posts_mid - wanted_post

        norm = np.linalg.norm(vec_post_to_mid)
        norm = 1 if norm == 0 else norm
        vec_post_to_mid = vec_post_to_mid / norm

        target = wanted_post + vec_post_to_mid

        return target

    def run(self, target: np.array, client: Client):
        ball: np.array = client.ball
        
        # Compute pos to go to before ball
        preshoot_pos = Striker.compute_preshoot_pos(target, ball)

        # Dynamically go towards pre-shoot position at least once
        last_ball_pos: np.array = ball
        while not np.isclose(self.__robot.position, preshoot_pos[:2]).all():
            self.__robot.goto(preshoot_pos, wait=False)

            # If ball moved, recompute pre shoot position
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
