FROM lgecloudroboticstask/ros:alpine-ros

ENV HOSTNAME cloud_bridge

RUN mkdir /root/src
RUN apk add --no-cache git bash
RUN git clone -b ${ROS_DISTRO} https://github.com/lge-ros2/cloud_bridge /root/src/cloud_bridge/
WORKDIR /root/src

RUN rosdep update
RUN rosdep install -y -r -q --from-paths cloud_bridge --ignore-src --rosdistro ${ROS_DISTRO}

# install colcon
RUN apk add --no-cache python3-dev py3-pip build-base zeromq-dev\
    && pip3 install -U setuptools colcon-common-extensions colcon-ros-bundle \
    && rm -rf /var/cache/apk/*

RUN ["/bin/bash", "-c", "source /usr/ros/${ROS_DISTRO}/setup.sh; colcon build --packages-up-to cloud_bridge"]

COPY ./docker/config/server.yaml ./docker/config/client.yaml ./docker/config/params.yaml /opt/
COPY ./_entrypoint.sh /

ARG cloud_ip

ENV BRIDGE_SERVER=TRUE
ENV CLOUD_IP=$cloud_ip
ENV MANAGE_PORT=26565
ENV manage_port=26565
ENV ROS_DOMAIN_ID=0
ENV ROBOT_NAME=cloi

STOPSIGNAL SIGKILL

RUN chmod +x /_entrypoint.sh

CMD ["/bin/bash", "-c", "sh /_entrypoint.sh"]