from client import Client

with Client(host='127.0.0.1', key='') as client:
    for i in range(6):
        for _ in range(10):
            client.robots['blue'][i].control(0, 0, 0)
    for i in range(6):
        for _ in range(10):
            client.robots['yellow'][i].control(0, 0, 0)
