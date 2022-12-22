import time

from client import Client

with Client(host='127.0.0.1', key='') as client:
    # client.robots['green'][1].kick()

    while True:
        print(f"ball: {client.ball}")

        for robot in client.robots["blue"].values():
            print(robot)
