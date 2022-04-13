# Semester_Project_Lassa

## Setup:
This runs in the combination with the simulator, for this get:
(make sure to clone the `develop` branch)
``` bash
https://github.com/epfl-lasa/simulator-backend/tree/develop/pybullet_ros2
```

``` bash
aica-docker interactive aica-technology/ros2-simulator:galactic -u ros2 --net host --no-hostname --ros-domain-id 0
```

``` bash
ros2 launch pybullet_ros2 franka.launch.py
```


# Run this container
<!-- ``` bash -->
<!-- docker run <…> -v /absolute/path/on/host: -->
<!-- ``` -->

# Run
Build the docker
``` bash
./build-server.sh
```

Run docker with temporary-python container:
``` bash
aica-docker interactive control-libraries-ros-demos:galactic-devel -u ros2 -v ${PWD}/python:/home/ros2/ros2_ws/src/combined_approach/python --net host --no-hostname --ros-domain-id 0
```
[WARNING] Make sure that this is run in the base-folder.

Run demo script:
``` bash
ros2 launch combined_approach demo.launch.py demo:=cartesian_twist_control
```

Run file
``` bash
aica-docker interactive control-libraries-ros-demos:galactic-devel -u ros2 --net bridge
```

# Make Sure to Install the Control Libraries
``` bash
git clone https://github.com/epfl-lasa/control-libraries
```

### Install control-libraries (skip this stage if already done)
``` bash
sudo bash control-libraries/source/install.sh
```

### Install the bindings using the pip installer
``` bash
export OSQP_INCLUDE_DIR='/usr/local/include/osqp'  # default 
export OPENROBOTS_INCLUDE_DIR='/opt/openrobots/include'  # default 
```

### Install the library in you python environment:
``` bash
pip3 install control-libraries/python
```




