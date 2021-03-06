ARG ROS_VERSION=galactic
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION}

# upgrade ament_cmake_python
RUN sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-ament-cmake-python && sudo rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${HOME}/lib
WORKDIR ${HOME}/lib

# Dynamic Obstacle Avoidance Library
RUN echo 31
RUN git clone -b main --single-branch https://github.com/epfl-lasa/dynamic_obstacle_avoidance
RUN  python3.8 -m pip install -r dynamic_obstacle_avoidance/requirements.txt
RUN cd dynamic_obstacle_avoidance && sudo python3.8 -m pip install --editable .

# Various Tools Library
RUN git clone -b main --single-branch https://github.com/hubernikus/various_tools
RUN python3.8 -m pip install -r various_tools/requirements.txt
RUN cd various_tools && sudo python3.8 -m pip install --editable .

# Files are copied indivually to allow compatibility
# for combo and without docker container
# This should be changed for production
RUN mkdir -p /home/${USER}/ros2_ws/src/combined_approach
WORKDIR /home/${USER}/ros2_ws/src/combined_approach
COPY --chown=${USER} ../setup.cfg .
COPY --chown=${USER} ../CMakeLists.txt ../package.xml .
COPY --chown=${USER} ../launch ./launch
COPY --chown=${USER} ../rviz ./rviz
COPY --chown=${USER} ../include ./include
COPY --chown=${USER} ../scripts ./scripts
COPY --chown=${USER} ../src ./src
COPY --chown=${USER} ../combined_approach ./combined_approach

WORKDIR /home/${USER}/ros2_ws/
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install"

WORKDIR /home/${USER}/ros2_ws/src/combined_approach
