from client import Client

with Client(host='127.0.0.1', key='') as client:
    robot = client.robots["blue"][0]
    print(client.ball)
    print(robot)
    robot.goto(lambda: (0., 0., 0.), wait=True)
