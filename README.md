# epos_controller

ROS wrapper for the MAXON EPOS motor controllers

**Note:** The repository development was terminated. Due to this, some functionalities are lacking (restarting controllers, configuration via config file, etc.)

## Building from Source

[![Source build and test](https://github.com/pawelir/epos_controller/actions/workflows/colcon-build-test.yaml/badge.svg)](https://github.com/pawelir/epos_controller/actions/workflows/colcon-build-test.yaml)

### Dependencies

- [std_msgs] - ROS standard messages

### Building

To build from source, clone the latest version from the repository into your workspace and compile the package using:

```bash
cd workspace/src
git clone https://github.com/pawelir/epos_controller.git
cd ../
rosdep update
rosdep install -i --from-paths src -y
colcon build --symlink-install
```

_**Note:** ROS should be sourced before: `source /opt/ros/<your-distro>/setup.bash`_

## Usage

Launch the core functionality with:

```bash
roslaunch epos_controller epos_controller.launch
```

### Launch files

- **epos_controller.launch** - Launches EPOS controller

## Nodes

### epos_controller

ROS wrapper for MAXON EPOS motor controller

#### Subscribed Topics

- **`/my_robot/epos_cmd_vel`** _std_msgs/msg/Int32MultiArray_

#### Service servers

- **`/epos_controller/stop_motors`** _std_srvs/srv/Trigger_ - Stop motors

  ```bash
  rosservice call /epos_controller/stop_motors std_srvs/srv/Trigger "{}"
  ```

- **`/epos_controller/stop_motors`** _std_srvs/srv/Trigger_ - Stop motors

  ```bash
  rosservice call /epos_controller/disable_motors std_srvs/srv/Trigger "{}"
  ```
