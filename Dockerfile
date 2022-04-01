ARG ROS_VERSION=galactic
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION}

# upgrade ament_cmake_python
RUN sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-ament-cmake-python && sudo rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${HOME}/lib
WORKDIR ${HOME}/lib

# Dynamic Obstacle Avoidance Library
RUN git clone -b main --single-branch https://github.com/epfl-lasa/dynamic_obstacle_avoidance
RUN python3 -m pip install -r dynamic_obstacle_avoidance/requirements.txt
RUN sudo python3 -m pip install --editable ./dynamic_obstacle_avoidance

# Various Tools Library
RUN echo 16
RUN git clone -b main --single-branch https://github.com/hubernikus/various_tools
RUN python3 -m pip install -r various_tools/requirements.txt
# RUN mv various_tools/src/* various_tools && rm various_tools/src -d
RUN sudo python3 -m pip install --editable ./various_tools
# RUN cd  && sudo python3 -m pip install --editable .

# WORKDIR /home/${USER}/ros2_ws
# COPY --chown=${USER} ../combined_approach ./src/combined_approach
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install"
