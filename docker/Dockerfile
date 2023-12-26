# Dockerfile written by SW 20210120

FROM ros:foxy-ros-base

SHELL ["/bin/bash", "-c"]

ENV HOSTNAME cloud_bridge

RUN sudo apt-get update -y && sudo apt-get install -y libzmq3-dev vim

RUN mkdir /root/src
RUN git clone -b foxy-dunfell https://github.com/lge-ros2/cloud_bridge /root/src/cloud_bridge/

COPY ./src /root/src/cloud_bridge/src/
COPY ./config/server.yaml /root/src/cloud_bridge/config/
COPY ./config/client.yaml /root/src/cloud_bridge/config/
COPY ./config/params.yaml /root/src/cloud_bridge/config/

WORKDIR /root

RUN apt update -y && apt upgrade -y && rosdep update 
RUN rosdep install -y -r -q --from-paths ./src/cloud_bridge --ignore-src --rosdistro $ROS_DISTRO
RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --packages-up-to cloud_bridge"]

COPY ./_entrypoint.sh /

ENV BRIDGE_SERVER=TRUE
ENV cloud_ip=$cloud_ip
ENV manage_port=25565
ENV ROS_DOMAIN_ID=0
ENV robot_name=cloi

STOPSIGNAL SIGINT
STOPSIGNAL SIGSTOP
STOPSIGNAL SIGKILL

RUN chmod +x /_entrypoint.sh

# ENTRYPOINT ["/_entrypoint.sh"]
