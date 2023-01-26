import fcntl
import json
import os
import signal
import socket
from enum import Enum
from typing import *

import numpy as np
import zmq
import threading
import time

import constants
import utils


class KICK(Enum):
    NO_KICK = 0
    STRAIGHT_KICK = 1
    CHIP_KICK = 2


configurations = {
    "dots": [
        ["green", 1, (constants.field_length / 4, -constants.field_width / 4, np.pi)],
        ["green", 2, (constants.field_length / 4, constants.field_width / 4, np.pi)],
        ["blue", 1, (-constants.field_length / 4, constants.field_width / 4, 0)],
        ["blue", 2, (-constants.field_length / 4, -constants.field_width / 4, 0)],
    ],
    "game": [
        ["green", 1, (constants.field_length / 4, 0, np.pi)],
        ["green", 2, (constants.field_length / 2, 0, np.pi)],
        ["blue", 1, (-constants.field_length / 4, 0, 0)],
        ["blue", 2, (-constants.field_length / 2, 0, 0)],
    ],
    "game_green_positive": [
        ["green", 1, (constants.field_length / 4, 0, np.pi)],
        ["green", 2, (constants.field_length / 2, 0, np.pi)],
        ["blue", 1, (-constants.field_length / 4, 0, 0)],
        ["blue", 2, (-constants.field_length / 2, 0, 0)],
    ],
    "game_blue_positive": [
        ["green", 1, (-constants.field_length / 4, 0, 0)],
        ["green", 2, (-constants.field_length / 2, 0, 0)],
        ["blue", 1, (constants.field_length / 4, 0, np.pi)],
        ["blue", 2, (constants.field_length / 2, 0, np.pi)],
    ],
    "side": [
        ["green", 1, (0.2, constants.field_width / 2, -np.pi / 2)],
        ["green", 2, (0.6, constants.field_width / 2, -np.pi / 2)],
        ["blue", 1, (-0.2, constants.field_width / 2, -np.pi / 2)],
        ["blue", 2, (-0.6, constants.field_width / 2, -np.pi / 2)],
    ],
    "swap_covers_green_positive": [
        ["green", 1, (0.09, -0.2, np.pi)],
        ["green", 2, (0.09, 0.2, np.pi)],
        ["blue", 1, (-0.09, -0.2, 0)],
        ["blue", 2, (-0.09, 0.2, 0)],
    ],
    "swap_covers_blue_positive": [
        ["green", 1, (-0.09, -0.2, 0)],
        ["green", 2, (-0.09, 0.2, 0)],
        ["blue", 1, (0.09, -0.2, np.pi)],
        ["blue", 2, (0.09, 0.2, np.pi)],
    ],
    "gently_swap_side": [
        ["green", 1, (0, -0.15, 0)],
        ["green", 2, (0, 0.5, 0)],
        ["blue", 1, (0, -0.5, np.pi)],
        ["blue", 2, (0, 0.15, np.pi)],
    ],
}


class ClientError(Exception):
    pass


class ClientTracked:
    def __init__(self):
        self.position: Optional[np.ndarray] = None
        self.pose: Optional[np.ndarray] = None
        self.orientation: Optional[float] = None
        self.last_update: Optional[int] = None


class ClientRobot(ClientTracked):
    def __init__(self, color, number, client):
        super().__init__()
        self.moved: bool = False
        self.color: str = color
        self.team: str = color
        self.number: int = number
        self.client: Client = client
        self.infrared: bool = False

        self.x_max: float = constants.field_length / 2 + constants.border_size / 2.0
        self.x_min: float = -self.x_max
        self.y_max: float = constants.field_width / 2 + constants.border_size / 2.0
        self.y_min: float = -self.y_max

        # PID controller
        self.old_e: np.ndarray = np.array([0, 0, 0])
        self.integral: np.ndarray = np.array([0, 0, 0])

    def ball(self):
        return self.client.ball

    def has_position(self, skip_old):
        seen = (self.position is not None) and (self.orientation is not None)
        if skip_old:
            seen = seen and self.age() < 1

        return seen

    def age(self):
        if self.last_update is None:
            return None

        return time.time() - self.last_update

    def kick(self, power=1., chip_kick=False):
        if chip_kick:
            self.client.command(number=self.number, power=power, kick=KICK.CHIP_KICK)
        else:
            self.client.command(number=self.number, power=power, kick=KICK.STRAIGHT_KICK)

    def dribble(self, speed):
        return self.client.command(number=self.number, dribbler=speed)

    def control(self, dx, dy, dturn):
        self.moved = True

        return self.client.command(number=self.number, forward_velocity=dx, left_velocity=dy, angular_velocity=dturn)

    # def leds(self, r, g, b):
    #     return self.client.command(self.color, self.number, "leds", {"r": r, "g": g, "b": b})

    def goto(self, target, wait=True, skip_old=True, pid_mode=False):
        if wait:
            while not self.goto(target, wait=False):
                time.sleep(0.05)
            self.control(0, 0, 0)
            return True

        arrived, order = self.goto_compute_order(target, skip_old, pid_mode=pid_mode)
        self.control(*order)

        return arrived

    def goto_compute_order(self, target, skip_old=True, pid_mode=False):
        p = 3
        i = 0
        d = 0

        if pid_mode:
            # zeigler nichols pid tuning method
            k_max = 0.018
            f_0 = 1.8

            p = 0.6 * k_max
            i = 2.0 * f_0
            d = 0.125 / f_0

        if callable(target):
            target = target()

        x, y, orientation = target
        # x = min(self.x_max, max(self.x_min, x))
        # y = min(self.y_max, max(self.y_min, y))
        Ti = utils.frame_inv(utils.robot_frame(self))
        target_in_robot = Ti @ np.array([x, y, 1])

        e = np.array([target_in_robot[0], target_in_robot[1], utils.angle_wrap(orientation - self.orientation)])

        self.integral = self.integral * e
        derivative = e - self.old_e
        self.old_e = e

        arrived = np.linalg.norm(e) < 0.05
        order = np.array([p, p, 1.5]) * e + d * derivative + i * self.integral

        return arrived, order

    def __str__(self):
        return f"Robot(team: {self.team}, id: {self.number}, position: {self.position}, orientation: {self.orientation})"


class Client:
    def __init__(self, host="127.0.0.1", key="", is_yellow=False, wait_ready=True):
        self.host = host
        self.running = True
        self.lock = threading.Lock()
        self.robots: Dict[str, Dict[int, ClientRobot]] = {}
        self.data_port = constants.yellow_data_port if is_yellow else constants.blue_data_port
        self.send_port = constants.yellow_send_port if is_yellow else constants.blue_send_port

        # Creating self.green1, self.green2 etc.
        for color, number in utils.all_robots():
            robot_id = utils.robot_list2str(color, number)
            robot = ClientRobot(color, number, self)
            self.__dict__[robot_id]: ClientRobot = robot

            if color not in self.robots:
                self.robots[color] = {}
            self.robots[color][number] = robot

        # Custom objects to track
        self.objs = {n: ClientTracked() for n in range(1, 9)}

        self.ball = None

        # ZMQ Context
        self.context = zmq.Context()

        # Creating subscriber connection
        self.recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_socket.bind(('0.0.0.0', self.data_port))
        fcntl.fcntl(self.recv_socket, fcntl.F_SETFL, os.O_NONBLOCK)

        self.on_update = None
        self.sub_packets = 0
        self.sub_thread = threading.Thread(target=lambda: self.sub_process())
        self.sub_thread.start()

        # Informations from referee
        self.referee = None

        # Creating request connection

        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_socket.connect((self.host, self.send_port))

        # Waiting for the first packet to be received, guarantees to have robot state after
        # client creation
        dt = 0.05
        t = 0
        warning_showed = False
        while wait_ready and self.sub_packets < 1:
            t += dt
            time.sleep(dt)
            if t > 3 and not warning_showed:
                warning_showed = True
                print("WARNING: Still no message from vision after 3s")
                print("if you want to operate without vision, pass wait_ready=False to the client")

    def __enter__(self):
        return self

    def __exit__(self, type, value, tb):
        self.stop()

    def update_position(self, tracked, infos):
        tracked.position = np.array(infos["position"])
        tracked.orientation = infos["orientation"]
        if "feedback" in infos and infos["feedback"] is not None:
            tracked.infrared = infos["feedback"]["infrared"]
        tracked.pose = np.array(list(tracked.position) + [tracked.orientation])
        tracked.last_update = time.time()

    def sub_process(self):
        last_t = time.time()
        while self.running:
            try:
                data = json.loads(self.recv_socket.recv(4096))
                ts = time.time()
                dt = ts - last_t
                last_t = ts

                if "ball" in data:
                    self.ball = None if data["ball"] is None else np.array(data["ball"])

                for team in utils.robot_teams():
                    if team in data:
                        for number, robot in enumerate(data[team]):
                            if robot is not None and "robot" in robot:
                                self.update_position(self.robots[team][number], robot["robot"])
                if "field" in data:
                    self.referee = data["field"]

                # if "referee" in json:
                #     self.referee = json["referee"]

                if self.on_update is not None:
                    self.on_update(self, dt)

                self.sub_packets += 1
            except socket.error as e:
                pass

    def stop_motion(self):
        for color in self.robots:
            robots = self.robots[color]
            for index in robots:
                if robots[index].moved:
                    try:
                        robots[index].control(0.0, 0.0, 0.0)
                    except ClientError:
                        pass

    def em(self):
        self.stop_motion()

    def stop(self):
        self.stop_motion()
        self.running = False

    def command_to_json(self, number, forward_velocity=0.0, left_velocity=0.0, angular_velocity=0.0,
                        kick=KICK.NO_KICK, charge=False, power=0.0, dribbler=0.0):
        if kick == KICK.STRAIGHT_KICK:
            return {
                "Command": {
                    "id": number,
                    "forward_velocity": forward_velocity,
                    "left_velocity": left_velocity,
                    "angular_velocity": angular_velocity,
                    "charge": charge,
                    "kick": {"StraightKick": {"power": power}},
                    "dribbler": dribbler
                }
            }
        elif kick == KICK.CHIP_KICK:
            return {
                "Command": {
                    "id": number,
                    "forward_velocity": forward_velocity,
                    "left_velocity": left_velocity,
                    "angular_velocity": angular_velocity,
                    "charge": charge,
                    "kick": {"ChipKick": {"power": power}},
                    "dribbler": dribbler
                }
            }
        return {
            "Command": {
                "id": number,
                "forward_velocity": forward_velocity,
                "left_velocity": left_velocity,
                "angular_velocity": angular_velocity,
                "charge": charge,
                "dribbler": dribbler
            }
        }

    def command(self, number, forward_velocity=0.0, left_velocity=0.0, angular_velocity=0.0,
                kick=KICK.NO_KICK, charge=False, power=0.0, dribbler=0.0):
        if threading.current_thread() is threading.main_thread():
            sigint_handler = signal.getsignal(signal.SIGINT)
            signal.signal(signal.SIGINT, signal.SIG_IGN)

        data = [self.command_to_json(number, forward_velocity, left_velocity, angular_velocity,
                             kick, charge, power, dribbler)]
        print(data)
        # self.lock.acquire()
        self.send_socket.sendall(json.dumps(data).encode())
        # self.lock.release()

        if threading.current_thread() is threading.main_thread():
            signal.signal(signal.SIGINT, sigint_handler)

        time.sleep(0.01)

    def goto_configuration(self, configuration_name="side", wait=False):
        targets = configurations[configuration_name]

        arrived = False
        while not arrived:
            arrived = True
            for color, index, target in targets:
                robot = self.robots[color][index]
                try:
                    arrived = robot.goto(target, wait=wait) and arrived
                except ClientError:
                    pass

        self.stop_motion()
