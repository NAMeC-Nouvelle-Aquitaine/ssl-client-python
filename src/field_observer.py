import numpy as np

from client import ClientRobot


class FieldObserver:
    @classmethod
    def guess_goal_keeper(cls, robots: dict[int, ClientRobot], goal_posts: np.array):
        """
        Guess who is the goalkeeper in the enemy team

        Parameters :
            goal_posts | xy coordinates of left and right enemy goal posts
            robots     | List of ClientRobot objects

        Returns :
            The ClientRobot that is closest to the middle of the goal posts
        """

        # Compute middle of goal posts
        goal_middle: np.array = (goal_posts[0] + goal_posts[1]) / 2

        # Contains the distance of each robot from the target post
        rob_dists: dict = {}
        for rob in robots.values():
            rob_dists[rob] = np.linalg.norm(goal_middle - rob.position)

        # Considering that the only enemy important to take in account is the closest
        guessed_goal_keeper = min(rob_dists, key=rob_dists.get)

        return guessed_goal_keeper

    @classmethod
    def get_furthest_post_of_gk(cls, gk: ClientRobot, goal_posts: np.array):
        """
        Determine which post is the farthest from the goalkeeper

        Parameters :
            goal_posts | xy coordinates of left and right goal posts
            gk         | The goalkeeper ClientRobot object

        Returns :
            The xy coordinates of the post from which the goalkeeper is farthest
        """
        # Compute the distance from the goalkeeper to each post
        gk_dists_from_posts: np.array = [np.linalg.norm(gk.position - post) for post in goal_posts]
        wanted_post: np.array = \
            np.maximum(*gk_dists_from_posts)

        return wanted_post
