from striker import Striker
from client import Client
import numpy as np

with Client(host='127.0.0.1', key='') as client:
    striker: Striker = Striker(client.robots["blue"][0])
    striker.run(
        np.array([4.5, 0]),
        client
    )
