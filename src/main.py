from client import Client

with Client(host='127.0.0.1', key='') as client:
    # while True:
    client.robots["blue"][0].control(0., 1., 1.)

    while True:
        pass

    # while True:
    #     print(f"ball: {client.ball}")
    #     client.robots["blue"][0].kick()
    #     # for robot in client.robots["blue"].values():
