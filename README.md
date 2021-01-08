# cloud_bridge (dashing version)

ROS2 message bridge between networks using zmq library

## Prerequisite

```shell
sudo apt-get install libzmq3-dev
```
## Build

Please setup ROS2 environment first!

```shell
source /opt/ros/dashing/setup.bash
colcon build --packages-up-to cloud_bridge
```

## Usage

### common configuration
set up ros2 configuration (ex. topic url, msg type, tf, etc.)
```shell
config/param.yaml
```
#### topic
```shell
  scan:
    topic: "scan"
    msg: "sensor_msgs/msg/LaserScan"
```
#### tf
```shell
  map_odom:
    base_frame: "map"
    child_frame: "odom"
```

### server
set up manage port and message port for zmq.
set up ros2 datas refer to param.yaml
#### configuration
```shell
config/server.yaml
```
#### run server
```shell
ros2 launch cloud_bridge cloud_bridge_server.launch.py
```
#### run server with commandline param

```shell
ros2 launch cloud_bridge cloud_bridge_server.launch.py manage_port:=25565 sub_port:=25567 pub_port:=25568
```
### client
set up server ip and manage port for server connect.
set up ros2 datas refer to param.yaml
#### configuration
```shell
config/client.yaml
```
#### run client

```shell
ros2 launch cloud_bridge cloud_bridge_client.launch.py
```
#### run client with commandline param

```shell
ros2 launch cloud_bridge cloud_bridge_client.launch.py cloud_ip:=127.0.0.1 manage_port:=25565
```
## Version info

- Please refer to each branch for ROS2 distro-version you want
  - [dashing](https://github.com/lge-ros2/cloud_bridge/tree/dashing)
  - [foxy](https://github.com/lge-ros2/cloud_bridge/tree/foxy)
