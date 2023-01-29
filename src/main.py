from client import Client
from basic_avoid import basic_avoid

"""
Side notes about the avoidance implementation :

    We need to pre-calculate areas or blobs to avoid to improve calculation
    
    If a robot is way too close to the robot to move ( < 0.01m ) the algorithm crashes and doesn't find any waypoint.
    Can be fixed by modifying the move vectors to go in all 4 directions in function space_away_from_circle()
"""


def main():
    with Client(host='127.0.0.1', key='') as client:
        while True:
            basic_avoid.step(client)


if __name__ == '__main__':
    main()
