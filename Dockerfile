ARG ROS_VERSION=galactic
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION}

# upgrade ament_cmake_python
RUN sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-ament-cmake-python && sudo rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${HOME}/lib
WORKDIR ${HOME}/lib

# Dynamic Obstacle Avoidance Library
RUN echo 25
RUN git clone -b main --single-branch https://github.com/epfl-lasa/dynamic_obstacle_avoidance
RUN  python3.8 -m pip install -r dynamic_obstacle_avoidance/requirements.txt
# RUN cd dynamic_obstacle_avoidance && sudo python3 -m pip install ----editable .
# RUN cd dynamic_obstacle_avoidance && python3 -m pip install --user --editable .
RUN cd dynamic_obstacle_avoidance && sudo python3.8 -m pip install --editable .

# Various Tools Library
RUN git clone -b main --single-branch https://github.com/hubernikus/various_tools
RUN python3.8 -m pip install -r various_tools/requirements.txt
RUN cd various_tools && sudo python3.8 -m pip install --editable .


WORKDIR /home/${USER}/ros2_ws
COPY --chown=${USER} ../combined_approach ./src/combined_approach
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install"
