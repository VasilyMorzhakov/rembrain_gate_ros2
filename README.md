# rembrain_gate_ros2
A gate to push/pull data from ROS2 to Rembrain backend

Required env variables to start rembrain_gate_ros2:
* WEBSOCKET_GATE_URL - websocker gate rembrain address
* ROBOT_NAME - the name of the robot in the rembrain system
* RRF_USERNAME - the name of the account in the rembraim system
* RRF_PASSWORD - rembraim account password

START
```
colcon build
source install/local_setup.sh
ros2 run rembrain_gate_ros2 gate --ros-args -p "in:=['commands__json']" -p "out:=['camera0__jpgpng', 'state__json']"
```

With the help of parameters when starting the module ros2 it is possible to configure the queues from which you need to receive data and the queues into which you need to put the. Specify the type of data to be transmitted

In the queue name what is before __ queue name in ros and ws-gate (they must match)

