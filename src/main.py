from client import Client
from src.basic_avoid import basic_avoid


def main():
    client = Client(host='127.0.0.1')
    basic_avoid.run(client)


if __name__ == '__main__':
    main()
