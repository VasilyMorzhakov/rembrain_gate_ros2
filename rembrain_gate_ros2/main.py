import os
import logging

import numpy
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String

import os
from rembrain_robot_framework import RobotProcess
from sensor_msgs.msg import Image

import numpy as np
rclpy.init(args=None)

class Pub(Node):
    def __init__(self):
        super().__init__('rembrain_gate_ros2')
        self.publisher_ = self.create_publisher(String, 'commands', 10)

    def pub(self, msg):
        self.publisher_.publish(msg)


class Sub(RobotProcess):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node = Node("sub")
        self.subscription = self.node.create_subscription(
            Image,
            'realsense_frame',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        self.publish(msg.data)

    def run(self) -> None:
        rclpy.spin(self.node)

class StateReceiver(RobotProcess):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node = Node("StateReceiver")
        self.subscription = self.node.create_subscription(
            String,
            'robot_state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        self.publish(msg.data.encode("utf-8"))

    def run(self) -> None:
        rclpy.spin(self.node)



class CommandWorker(RobotProcess):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pub = Pub()

    def run(self) -> None:
        self.log.info(f"{self.name} robot worker started")

        while True:
            if not self.consume_queues["commands_ros"].empty():
                command: dict = self.consume(queue_name="commands_ros")
                msg = String()
                msg.data = json.dumps(command)
                self.pub.pub(msg)

class DepthMixin(RobotProcess):
    def run(self):
        while True:
            ar=self.consume()
            w1=int.from_bytes(ar[0:4].tobytes(),'little')
            h1 = int.from_bytes(ar[4:8].tobytes(), 'little')
            w2 = int.from_bytes(ar[8:12].tobytes(), 'little')
            h2=int.from_bytes(ar[12:16].tobytes(),'little')
            img=numpy.frombuffer(ar[16:16+w1*h1*3], dtype=numpy.uint8)
            img=img.reshape((h1,w1,3))
            depth=numpy.frombuffer(ar[16+w1*h1*3:], dtype=numpy.uint16)
            depth=depth.reshape((h2,w2))
            self.publish((img, depth,{}))


import os

from rembrain_robot_framework import RobotDispatcher
from rembrain_robot_framework.processes import WsRobotProcess, VideoPacker



def main_func(args=None):
    process_map = {
        "sensor_sender": WsRobotProcess,
        "command_receiver": WsRobotProcess,
        "command_worker": CommandWorker,
        "image_receiver": Sub,
        "depth_mixin": DepthMixin,
        "video_packer": VideoPacker,
        "video_streamer": WsRobotProcess,
        "state_receiver": StateReceiver
    }

    config = {
        "processes": {
            "sensor_sender":{
                "command_type": "push",
                "exchange": "state",
                "consume": ["robot_to_websocket"]
            },
            "state_receiver": {
                "publish": ["robot_to_websocket"]
            },
            "command_receiver": {
                "command_type": "pull",
                "exchange": "commands",
                "data_type": "json",
                "publish": ["commands_ros"]
            },
            "command_worker": {
                "consume": ["commands_ros"]
            },
            "image_receiver": {
                "publish": ["image_ros"]
            },
            "depth_mixin": {
                "consume": ["image_ros"],
                "publish": ["to_pack"]
            },
            "video_packer": {
                "pack_type": "JPG_PNG",
                "consume": ["to_pack"],
                "publish": ["image_processed"]
            },
            "video_streamer": {
                "command_type": "push_loop",
                "exchange": "camera0",
                "consume": ["image_processed"]
            }

        }
    }

    processes = {p: {"process_class": process_map[p]} for p in config["processes"]}

    robot_dispatcher = RobotDispatcher(config, processes, in_cluster=False)
    robot_dispatcher.start_processes()
    robot_dispatcher.run()

if __name__ == "__main__":
    main_func()

