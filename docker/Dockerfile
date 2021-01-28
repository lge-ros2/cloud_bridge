# Dockerfile written by SW 20210120

FROM ros:foxy-ros-base

SHELL ["/bin/bash", "-c"]

ENV HOSTNAME cloud_bridge

RUN sudo apt-get update -y && sudo apt-get install -y libzmq3-dev

RUN mkdir /root/src
RUN git clone -b foxy https://github.com/lge-ros2/cloud_bridge /root/src/cloud_bridge/
WORKDIR /root/src

RUN apt update -y && apt upgrade -y && rosdep update 
RUN rosdep install -y -r -q --from-paths cloud_bridge --ignore-src --rosdistro $ROS_DISTRO
RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --packages-up-to cloud_bridge"]

COPY ./config/server.yaml ./config/client.yaml ./config/params.yaml /opt/
COPY ./_entrypoint.sh /

ARG cloud_ip

ENV BRIDGE_SERVER=TRUE
ENV cloud_ip=$cloud_ip
ENV manage_port=25565
ENV ROS_DOMAIN_ID=0
ENV robot_name=cloi

STOPSIGNAL -9
STOPSIGNAL SIGINT

ENTRYPOINT ["/_entrypoint.sh"]