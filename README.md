# 3 DOF Robot Controller Simulation
This repository contains a simulation of a 3 Degrees of Freedom (DOF) robotic arm controller. The project aims to simulate a controller for a 3 DOF robot, enabling it to reach a specified setpoint by controlling its joint velocities through Jacobian matrices.

## Project Overview
This simulation project uses the **Robotics Toolbox for Python** to modeland control a 3 DOF robot. The main functionalities include:

- Visualized the robot's with 3D model in **RVIZ2**.

- Generating a 3 DOF robot model using the **xacro urdf** file.
- Controlling the robot's via **custom teleop keyboard** inside the packages.
- Controlling the robot's end-effector by computing joint velocities using the **Jacobian**.
- Random target generation for the end-effector through a **ROS2 client-server** model.
- Custom target setpoint for the end-effectorthrough a **ROS2 client-server** model.
- Integration of ROS2 nodes for real-time robot control and simulation.

The repository includes the necessary code for setting up the robot, controlling it, and visualizing its movements in a simulation environment.

## Features

- **Jacobian-based Velocity Control:** The controller uses the Jacobian matrix to compute the necessary joint velocities to achieve a target (x, y, z) in the workspace.

- **Custom Teleoperation Keyboard:** Controlled your robot with only 1 keyboard.
- **ROS2 Integration:** A ROS2 client-server communication is established to generate random targets and update the robot's setpoint accordingly.
- **RVIZ Model Visualized:** Fast and easy visualizer with adjustable models.

## Preview

## System Architechture

## Prerequisites
To run this project, ensure you have the following installed:
### System
- [**ROS2 Humble** (or your preferred ROS2 distribution)](https://docs.ros.org/en/humble/Installation.html)
- **Colcon build system**
- **Python 3.8+**
- [**Robotics Toolbox for Python**](https://petercorke.github.io/robotics-toolbox-python/)
### ROS 2
- [**RVIZ2**](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html)
- [**geometry2 tf2_ros**](https://github.com/ros2/geometry2)
- [**robot_state_publisher package**](https://github.com/ros/robot_state_publisher)

## Installation
### Setup Environment
This project need a **python 3.8+** environment and ```Robotics Toolbox for Python``` library.

1. Make sure your environment use **python 3.8+** by using this command.

    ```bash
    python3 -V
    ```
2. Install & upgrade **python3-pip**.
    ```bash
    sudo apt-get install python3-pip
    python3 -m pip install --upgrade pip
    pip3 install -U pip
    ```
    Make sure pip is installed using `pip3 -V`

3. Install ```Robotics Toolbox for Python``` using pip3. Tutorial from [**Robotics Toolbox GitHub**](https://github.com/petercorke/robotics-toolbox-python).
    ```bash
    pip3 install roboticstoolbox-python
    ```
    After finished install make sure you installed **roboticstoolbox-python** successfully using
    ```bash
    pip3 show roboticstoolbox-python
    ```  
    For use ```Robotics Toolbox for Python``` have to use **numpy < 1.25.0**
    ```bash
    pip3 install numpy==1.24.4
    ```
    Make sure you using **numpy < 1.25.0** by
    ```bash
    pip3 show numpy
    ```
    Check that **roboticstoolbox-python** can use with python environment.
    ```sh
    teety@TeeTy-Ubuntu:~$ python3
    Python 3.10.12 (main, Sep 11 2024, 15:47:36) [GCC 11.4.0] on linux
    Type "help", "copyright", "credits" or "license" for more information.
    >>> import numpy as np
    >>> np.__version__
    '1.24.4'
    >>> import spatialmath as stm
    >>> stm.__version__
    '1.1.11'
    >>> import roboticstoolbox as rtb
    >>> rtb.__version__
    '1.1.0'
    >>> print(rtb.models.Panda())
    ERobot: panda (by Franka Emika), 7 joints (RRRRRRR), 1 gripper, geometry, collision
    |   link   |     link     |   joint   |    parent   |              ETS: parent to link               |
    |:--------:|:------------:|:---------:|:-----------:|:----------------------------------------------:|
    |     0    |  panda_link0 |           |     BASE    | SE3()                                          |
    |     1    |  panda_link1 |     0     | panda_link0 | SE3(0, 0, 0.333) ⊕ Rz(q0)                      |
    |     2    |  panda_link2 |     1     | panda_link1 | SE3(-90°, -0°, 0°) ⊕ Rz(q1)                    |
    |     3    |  panda_link3 |     2     | panda_link2 | SE3(0, -0.316, 0; 90°, -0°, 0°) ⊕ Rz(q2)       |
    |     4    |  panda_link4 |     3     | panda_link3 | SE3(0.0825, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3)       |
    |     5    |  panda_link5 |     4     | panda_link4 | SE3(-0.0825, 0.384, 0; -90°, -0°, 0°) ⊕ Rz(q4) |
    |     6    |  panda_link6 |     5     | panda_link5 | SE3(90°, -0°, 0°) ⊕ Rz(q5)                     |
    |     7    |  panda_link7 |     6     | panda_link6 | SE3(0.088, 0, 0; 90°, -0°, 0°) ⊕ Rz(q6)        |
    |     8    | @panda_link8 |           | panda_link7 | SE3(0, 0, 0.107)                               |
    >>> 
    [2]+  Stopped                 python3
    ```
    If you see a table, so now your **roboticstoolbox-python** library is ready now.

### Setup ROS2

1. Make sure your environment have [**ROS2 Humble** (or your preferred ROS2 distribution)](https://docs.ros.org/en/humble/Installation.html)

    ```sh
    printenv | grep -i ROS
    ```

    ```bash
    ROS_VERSION=2
    ROS_PYTHON_VERSION=3
    .
    .
    .
    ROS_DISTRO=humble
    ```

    if you see **humble** or other **ROS2 Distributions** maens your ROS2 is ready now.

2. Install ```RVIZ2``` ```TF2``` and ```robot_state_publisher package``` in your ROS2 Environment. With install ```ros-desktop-full```

    ```bash
    sudo apt install ros-{ROS_DISTRO}-desktop-full
    ```

    replace **{ROS_DISTRO}** with your ros2 distributions. **Example**

    ```bash
    sudo apt install ros-humble-desktop-full
    ```

3. Source your environment.

    ```bash
    source /opt/ros/humble/setup.bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

4. Check a require ```packages``` in ROS2.

    ```bash
    ros2 pkg executables rviz2
    ros2 pkg executables tf2_ros
    ros2 pkg executables robot_state_publisher 
    ```

    ```bash
    rviz2 rviz2
    tf2_ros buffer_server
    tf2_ros static_transform_publisher
    tf2_ros tf2_echo
    tf2_ros tf2_monitor
    robot_state_publisher robot_state_publisher
    ```
5. Check a require ```interfaces``` in ROS2

    ```bash
    ros2 interface package sensor_msgs 
    ```

    ```bash
    sensor_msgs/msg/CameraInfo
    sensor_msgs/msg/Illuminance
    sensor_msgs/msg/NavSatStatus
    sensor_msgs/msg/PointCloud
    sensor_msgs/msg/LaserEcho
    sensor_msgs/msg/FluidPressure
    sensor_msgs/msg/BatteryState
    sensor_msgs/msg/Range
    sensor_msgs/msg/JoyFeedbackArray
    sensor_msgs/msg/Joy
    sensor_msgs/srv/SetCameraInfo
    sensor_msgs/msg/ChannelFloat32
    sensor_msgs/msg/NavSatFix
    sensor_msgs/msg/Temperature
    sensor_msgs/msg/Imu
    sensor_msgs/msg/RelativeHumidity
    sensor_msgs/msg/LaserScan
    sensor_msgs/msg/MultiDOFJointState
    sensor_msgs/msg/MultiEchoLaserScan
    sensor_msgs/msg/MagneticField
    sensor_msgs/msg/PointCloud2
    sensor_msgs/msg/CompressedImage
    sensor_msgs/msg/JoyFeedback
    sensor_msgs/msg/TimeReference
    sensor_msgs/msg/RegionOfInterest
    sensor_msgs/msg/JointState
    sensor_msgs/msg/Image
    sensor_msgs/msg/PointField
    ```

    **Check that you have all package above for next step.**

### Install Package
1. Go to ~ ```PATH``` and clone this repository:
    ```bash
    cd ~
    git clone https://github.com/TeeTyJunGz/FUN4.git
    ```
2. Navigate to the project directory:
    ```bash
    cd FUN4
    ```
3. Build and source the workspace:
    ```bash
    colcon build && source install/setup.bash
    ```
    **You should finished build with 2 packages.**
    ```bash
    Starting >>> robotic_interfaces
    Starting >>> robotics_model_3dof
    Finished <<< robotics_model_3dof [0.42s]                                    
    Finished <<< robotic_interfaces [0.71s]                  

    Summary: 2 packages finished [0.89s]
    ```
4. Add source to ```~/.bashrc```:
    ```bash
    echo "source ~/FUN4/install/setup.bash" >> ~/.bashrc
    ```
5. Source ```~/.bashrc```:
    ```bash
    source ~/.bashrc
    ```
6. Check ROS2 packages:
    ```bash
    ros2 pkg executables robotics_model_3dof
    ```
    **This is all ```packages``` from this project.**
    ```bash
    robotics_model_3dof TF_Check.py
    robotics_model_3dof cpp_node_test
    robotics_model_3dof kinematics.py
    robotics_model_3dof robot_controller.py
    robotics_model_3dof robot_scheduler.py
    robotics_model_3dof target_randomizer.py
    robotics_model_3dof teleop_keyboard.py
    ```
6. Check ROS2 interfaces:
    ```bash
    ros2 interface package robotic_interfaces
    ```
    **This is all ```interfaces``` from this project.**
    ```bash
    robotic_interfaces/srv/Keyboard
    robotic_interfaces/srv/RandomTarget
    robotic_interfaces/srv/StateScheduler
    ```

## Usage
### Workspace of 3DOF Robot
![workspace](https://raw.githubusercontent.com/TeeTyJunGz/FUN4/refs/heads/main/src/robotics_model_3dof/images/workspace.png)

This workspace generated from find limit of ```q1 q2 q3``` and sample all possible data that ```q1 q2 q3``` can go to that position, last plot all position to a graph.

### Launch Project
You can launch the project using the provided ROS2 launch file. The launch file is visualized a 3DOF robot in rviz2, that you can control later.

```bash
ros2 launch robotics_model_3dof robot.control.launch.py 
```

![rviz](https://raw.githubusercontent.com/TeeTyJunGz/FUN4/refs/heads/main/src/robotics_model_3dof/images/rviz_startup.png)


**You can random a visualized target ```Pose``` in this rviz2 with a service via terminal**

```bash
ros2 service call /rand_target robotic_interfaces/srv/RandomTarget "data: true" 
```
- ```data``` = true 

    Target will random spawned somewhere in the robot workspace, and this service will ```response``` a position that random too.

- ```data``` = false

    Target won't spawn if you request ```False``` and this service will do nothing.

**You can visualized an ```End Effector``` with controller mode too, with these command.**

Open a new terminal

```bash
ros2 run robotics_model_3dof teleop_keyboard.py 
```

**If your ```teleop_keyboard``` open correctly at launch file terminal will show Tele-operation Mode**

![launch_connect_teleop](https://raw.githubusercontent.com/TeeTyJunGz/FUN4/refs/heads/main/src/robotics_model_3dof/images/launch_connect_teleop.png)

**Now at rviz2 an ```End Effector Pose``` will appeared**

![rviz_eff](https://raw.githubusercontent.com/TeeTyJunGz/FUN4/refs/heads/main/src/robotics_model_3dof/images/rviz_eff.png)

### Robot Controller
**You can controlled robot using ```Cutom Service Interface``` or ```Teleop Keyboard```**

![custominterface](https://raw.githubusercontent.com/TeeTyJunGz/FUN4/refs/heads/main/src/robotics_model_3dof/images/CustomKeyInter.png)


- **```Custom Service Interfaces```**

    **Request**
    - ```string mode``` : Mode selector **[ IPK, Teleop Based, Teleop End Effector, Auto ]**
    
    - ```float64 x, y, z``` : Given position setpoint for **IPK** mode
    
    **Response**

    - ```bool success``` : Responsed **``True``** when can change mode and **``False``** when cannot change

    - ```string message``` : Responsed a message I tell requester about mode changes
    - ```string workspace``` : Responsed only when use **IPK** mode and float64 x, y, z is out workspace


- **```Teleop Keyboard```**
    
    Teleop Keyboard is just a Custom Service Interfaces with GUI, you can change mode same as Custom Service Interfaces. Input for IPK mode can also done within this keyboard

![teleoperation_keyboard](https://raw.githubusercontent.com/TeeTyJunGz/FUN4/refs/heads/main/src/robotics_model_3dof/images/teleoperation_Keyboard.png)