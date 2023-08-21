ARG ROS_DISTRO
ARG BASE_IMAGE

FROM $BASE_IMAGE
RUN mkdir /workspace && apt-get update && apt-get install python3-catkin-tools python3-pip -y
WORKDIR /workspace
RUN pip3 install pytransform3d
COPY ./../src /workspace/src
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y && catkin build apriltag_ros spot_viewpoint_planning"
