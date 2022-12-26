from client import Client
from src.basic_avoid import basic_avoid


def main():
    client = Client(host='127.0.0.1')
    basic_avoid.run(client)

"""
with Client(host='127.0.0.1', key='') as client:
    robot = client.robots["blue"][0]
    print(client.ball)
    print(robot)
    robot.goto(lambda: (0., 0., 0.), wait=True)
    robot.dribble(0.0)

    while True:
        if robot.infrared:
            robot.dribble(1000.0)
            robot.goto((np.cos(time.time()) * 1000, np.sin(time.time()) * 1000, .0), wait=False)
        else:
            robot.dribble(0.0)
            robot.control(.0, .0, .0)
"""

if __name__ == '__main__':
    main()
