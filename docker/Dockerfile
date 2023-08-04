ARG ROS_DISTRO
ARG BASE_IMAGE

FROM $BASE_IMAGE
RUN mkdir /workspace && apt-get update && apt-get install python3-catkin-tools python3-pip -y
WORKDIR /workspace
RUN pip3 install pytransform3d
