from client import Client
from basic_avoid import basic_avoid


def main():
    with Client(host='127.0.0.1', key='') as client:
        while True:
            basic_avoid.step(client)


if __name__ == '__main__':
    main()
