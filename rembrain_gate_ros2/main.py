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

import os

from rembrain_robot_framework import RobotDispatcher
from rembrain_robot_framework.processes import WsRobotProcess, VideoPacker

import numpy as np
rclpy.init(args=None)

class Pub(Node):
    def __init__(self, ros_exchange):
        super().__init__('rembrain_gate_ros2')
        self.publisher_ = self.create_publisher(String, ros_exchange, 10)

    def pub(self, msg):
        self.publisher_.publish(msg)


class Sub(RobotProcess):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node = Node("sub")
        self.type = kwargs["type"]
        self.consume_param = kwargs["consume_ros"][0]
        self.subscription = None
        if self.type == "img":
            self.subscription = self.node.create_subscription(
                Image,
                self.consume_param,
                self.listener_callback,
                10)
        elif self.type == "json":
            self.subscription = self.node.create_subscription(
                String,
                self.consume_param,
                self.listener_callback,
                10)


    def listener_callback(self, msg):
        if self.type == "img":
            self.publish(msg.data)
        elif self.type == "json":
            self.publish(msg.data.encode("utf-8"))

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
        self.consume_param = list(self.consume_queues.keys())[0]
        self.publish_param = kwargs["publish_ros"][0]
        self.pub = Pub(self.publish_param)

    def run(self) -> None:
        self.log.info(f"{self.name} robot worker started")

        while True:
            if not self.consume_queues[self.consume_param].empty():
                command: dict = self.consume(queue_name=self.consume_param)
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
            
class RgbMixin(RobotProcess):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.width=kwargs["width"]
        self.height=kwargs["height"]
    def run(self):
        while True:
            ar=self.consume()
            img=numpy.frombuffer(ar, dtype=numpy.uint8)
            img=img.reshape((self.height,self.width,3))
            self.publish((img, None,{}))


def main_func(args=None):
    param = Node("param")
    param.declare_parameter('in', None)
    in_param = param.get_parameter('in').get_parameter_value().string_array_value

    param.declare_parameter('out', None)
    out_param = param.get_parameter('out').get_parameter_value().string_array_value

    process_map = {}
    config = {"processes": {}}

    for i in range(len(in_param)):
        param = in_param[i].split(":")
        if param[2] == "json":
            process_map["command_receiver_" + str(i)] = WsRobotProcess
            process_map["command_worker_" + str(i)] = CommandWorker
            config["processes"]["command_receiver_" + str(i)] = {
                    "command_type": "pull",
                    "exchange": param[1],
                    "data_type": "json",
                    "publish": ["commands_ros_" + str(i)]
                }
            config["processes"]["command_worker_" + str(i)] = {
                    "consume": ["commands_ros_" + str(i)],
                    "publish_ros": [param[0]]
                }

    for i in range(len(out_param)):
        param = out_param[i].split(":")
        if param[2] == "jpgpng":
            process_map["image_receiver_" + str(i)] = Sub
            process_map["depth_mixin_" + str(i)] = DepthMixin
            process_map["video_packer_" + str(i)] = VideoPacker
            process_map["video_streamer_" + str(i)] = WsRobotProcess

            config["processes"]["image_receiver_" + str(i)] = {
                "type": "img",
                "consume_ros": [param[0]],
                "publish": ["image_ros_" + str(i)]
            }
            config["processes"]["depth_mixin_" + str(i)] = {
                "consume": ["image_ros_" + str(i)],
                "publish": ["to_pack_" + str(i)]
            }
            config["processes"]["video_packer_" + str(i)] = {
                "pack_type": "JPG_PNG",
                "consume": ["to_pack_" + str(i)],
                "publish": ["image_processed_" + str(i)]
            }
            config["processes"]["video_streamer_" + str(i)] = {
                "command_type": "push_loop",
                "consume": ["image_processed_" + str(i)],
                "exchange": param[1]
            }
        elif param[2] == "jpg":
            process_map["image_receiver_" + str(i)] = Sub
            process_map["rgb_mixin_" + str(i)] = RgbMixin
            process_map["video_packer_" + str(i)] = VideoPacker
            process_map["video_streamer_" + str(i)] = WsRobotProcess

            config["processes"]["image_receiver_" + str(i)] = {
                "type": "img",
                "consume_ros": [param[0]],
                "publish": ["image_ros_" + str(i)]
            }
            config["processes"]["rgb_mixin_" + str(i)] = {
                "width": int(param[3]),
                "height": int(param[4]),
                "consume": ["image_ros_" + str(i)],
                "publish": ["to_pack_" + str(i)]
            }
            config["processes"]["video_packer_" + str(i)] = {
                "pack_type": "JPG",
                "consume": ["to_pack_" + str(i)],
                "publish": ["image_processed_" + str(i)]
            }
            config["processes"]["video_streamer_" + str(i)] = {
                "command_type": "push_loop",
                "consume": ["image_processed_" + str(i)],
                "exchange": param[1]
            }
        elif param[2] == "json":
            process_map["json_receiver_" + str(i)] = Sub
            process_map["json_streamer_" + str(i)] = WsRobotProcess

            config["processes"]["json_receiver_" + str(i)] = {
                "type": "json",
                "consume_ros": [param[0]],
                "publish": ["json_ros_" + str(i)]
            }
            config["processes"]["json_streamer_" + str(i)] = {
                "command_type": "push_loop",
                "consume": ["json_ros_" + str(i)],
                "exchange": param[1]
            }

    processes = {p: {"process_class": process_map[p]} for p in config["processes"]}

    robot_dispatcher = RobotDispatcher(config, processes, in_cluster=False)
    robot_dispatcher.start_processes()
    robot_dispatcher.run()

if __name__ == "__main__":
    main_func()

