from client import Client
from basic_avoid import basic_avoid


def main():
    client = Client(host='127.0.0.1')
    # Available values :
    #   TEST_EXPERIMENTS
    #       | Executes static scenarios to show proof that algorithm works
    #   INTERPRET_FROM_REAL
    #       | Place robots on specific positions,
    #       | where Robot 0 is the one to move,
    #       | Robot 1 is the robot to avoid,
    #       | Robot 2 is the destination point (please put a cover instead of a robot)
    basic_avoid.run(client, scenario="INTERPRET_FROM_REAL")


if __name__ == '__main__':
    main()
