import os
import logging
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
            'Image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        self.publish(msg.data)

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
            #print(np.array(self.consume()).reshape((224, 224, 3)))
            self.publish((np.array(self.consume()).reshape((224, 224, 3)), None))

import os

from rembrain_robot_framework import RobotDispatcher
from rembrain_robot_framework.processes import WsRobotProcess, VideoPacker



def main_func(args=None):
    process_map = {
        "command_receiver": WsRobotProcess,
        "command_worker": CommandWorker,
        "image_receiver": Sub,
        "depth_mixin": DepthMixin,
        "video_packer": VideoPacker,
        "video_streamer": WsRobotProcess
    }

    config = {
        "processes": {
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
                "pack_type": "JPG",
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

