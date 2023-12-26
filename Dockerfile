# FROM lgecloudroboticstask/lge-ros-base:humble-20230119

FROM ros:humble
LABEL maintainer="SW <sewan.gu@lge.com>"
LABEL intial_date="20230119"
# LABEL output_image="cloud_bridge:humble-offloading-20230508"
# LABEL output_image="cloud_bridge:humble-20230721"
LABEL output_image="cloud_bridge:humble-20230811"
# command : docker build --pull -t lgecloudroboticstask/cloud_bridge:humble-xxxx -f Dockerfile.cloud_bridge.humble .

SHELL ["/bin/bash", "-c"]


# RUN apt-get update -y && apt-get install -y libzmq3-dev curl apt-utils vim iputils-ping net-tools 
#openssh-server

#RUN ssh-keygen -q -t rsa -N '' -f /id_rsa

## due to ROS2 key changes 
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ubuntu

# RUN apt-get update -y && apt-get upgrade -y && rosdep update
# RUN sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-joy-teleop
# RUN rosdep update

RUN mkdir -p /home/ubuntu/cloud_bridge/src
RUN mkdir -p /home/ubuntu/py_srvcli/src

RUN mkdir -p /home/ubuntu/cloud_bridge/src/ebme_interfaces
WORKDIR /home/ubuntu/cloud_bridge/src/
RUN git clone -b $ROS_DISTRO https://github.com/lge-ros2/cloud_bridge 

# WORKDIR /home/ubuntu/cloud_bridge/src/cloud_bridge
# RUN git fetch
# RUN git pull

COPY ./ebme/ebme_interfaces /home/ubuntu/cloud_bridge/src/ebme_interfaces
COPY ./ebme/exinterface_msgs /home/ubuntu/cloud_bridge/src/exinterface_msgs
COPY ./ebme/slam_msgs /home/ubuntu/cloud_bridge/src/slam_msgs
COPY ./fms_interfaces /home/ubuntu/cloud_bridge/src/fms_interfaces
COPY ./sensor_interfaces /home/ubuntu/cloud_bridge/src/sensor_interfaces 


COPY ./config/server.yaml /home/ubuntu/cloud_bridge/src/cloud_bridge/config/
COPY ./config/client.yaml /home/ubuntu/cloud_bridge/src/cloud_bridge/config/
COPY ./config/params.yaml /home/ubuntu/cloud_bridge/src/cloud_bridge/config/

COPY ./config/server_offloading.yaml /home/ubuntu/cloud_bridge/config_offloading/server.yaml
COPY ./config/params_offloading.yaml /home/ubuntu/cloud_bridge/config_offloading/params.yaml
COPY ./config/client_offloading.yaml /home/ubuntu/cloud_bridge/config_offloading/client.yaml

COPY ./config/server_lgnav_offloading.yaml /home/ubuntu/cloud_bridge/config_lgnav_offloading/server.yaml
COPY ./config/params_lgnav_offloading.yaml /home/ubuntu/cloud_bridge/config_lgnav_offloading/params.yaml
COPY ./config/client_lgnav_offloading.yaml /home/ubuntu/cloud_bridge/config_lgnav_offloading/client.yaml

COPY ./py_srvcli /home/ubuntu/cloud_bridge/src/py_srvcli/
COPY ./publisher.py  /home/ubuntu/publisher.py
COPY ./subscriber.py  /home/ubuntu/subscriber.py

RUN sudo apt-get update -y && sudo apt-get install -y libzmq3-dev curl apt-utils vim iputils-ping net-tools 
RUN sudo apt upgrade -y 

RUN sudo apt-get install -y ros-$ROS_DISTRO-nav2-msgs ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-joy-teleop

WORKDIR /home/ubuntu/cloud_bridge
RUN rosdep update
RUN rosdep install -y -r -q --from-paths src --ignore-src --rosdistro $ROS_DISTRO
RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install"]

# WORKDIR /home/ubuntu/py_srvcli
# RUN rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
# RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --packages-select py_srvcli"]


COPY ./cloud_bridge_entrypoint.sh /

ARG cloud_ip="1.1.1.1"

ENV BRIDGE_SERVER=TRUE
ENV CLOUD_IP=$cloud_ip
ENV manage_port=26565
ENV MANAGE_PORT=26565
ENV ROS_DOMAIN_ID=172
ENV ROBOT_NAME=cloi
ENV HOSTNAME cloud_bridge
ENV MANAGE_PORT=26565

STOPSIGNAL SIGINT
STOPSIGNAL SIGSTOP
STOPSIGNAL SIGKILL

WORKDIR /home/ubuntu/cloud_bridge

RUN sudo chmod +x /cloud_bridge_entrypoint.sh
ENTRYPOINT ["/cloud_bridge_entrypoint.sh"]

