from client import Client
from basic_avoid import basic_avoid

"""
Side notes about the implementation :

    If this is meant to be used during the competition, the avoidance path computed
    if multiple robots form a wall. The current algorithm will try to pierce through that wall
    We need to pre-calculate areas or blobs to avoid
"""


def main():
    with Client(host='127.0.0.1', key='') as client:
        while True:
            basic_avoid.step(client)


if __name__ == '__main__':
    main()
