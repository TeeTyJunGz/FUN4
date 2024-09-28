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
This project need a **python 3.8+** environment and **Robotics Toolbox for Python** library.

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

3. Install **Robotics Toolbox for Python** using pip3. Tutorial from [**Robotics Toolbox GitHub**](https://github.com/petercorke/robotics-toolbox-python).
    ```bash
    pip3 install roboticstoolbox-python
    ```
    After finished install make sure you installed **roboticstoolbox-python** successfully using
    ```bash
    pip3 show roboticstoolbox-python
    ```  
    For use **Robotics Toolbox for Python** have to use **numpy < 1.25.0**
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
    If you see a table, so now your **roboticstoolbox-python** library is ready to use now.
## Usage
Instructions on how to use the project or its features.

## Contributing
Guidelines on how to contribute to the project.

## License
This project is licensed under the [License Name] - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments
Mention anyone who contributed to the project or inspired you. 