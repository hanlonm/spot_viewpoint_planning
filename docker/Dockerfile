ARG ROS_DISTRO
ARG BASE_IMAGE

FROM $BASE_IMAGE
RUN mkdir /workspace && apt-get update && apt-get install python3-catkin-tools -y
WORKDIR /workspace
