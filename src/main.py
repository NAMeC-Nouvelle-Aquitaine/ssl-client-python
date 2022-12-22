from client import Client

with Client(host='127.0.0.1', key='') as client:
    while True:
        robot = client.robots["blue"][0]
        print(client.ball)
        print(robot)
        robot.goto(lambda: (*client.ball, 0.), wait=True)

    # while True:
    #     print(f"ball: {client.ball}")
    #     client.robots["blue"][0].kick()
    #     # for robot in client.robots["blue"].values():
