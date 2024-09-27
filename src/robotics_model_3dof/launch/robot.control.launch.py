#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
    
def generate_launch_description():
    
    pkg = get_package_share_directory('robotics_model_3dof')
    rviz_path = os.path.join(pkg,'config','display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    #robot_desc_xml = xacro.process_file(path_description,mappings={'robot_name': namespace}).toxml()
    
    parameters = [{'robot_description':robot_desc_xml}]
    #parameters.append({'frame_prefix':namespace+'/'})
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher_gui)
    
    package_name = 'robotics_model_3dof'
    
    executable_name = ['target_randomizer', 'kinematics', 'robot_controller', 'robot_scheduler']
    for i in range(len(executable_name)):
        if executable_name[i] == 'target_randomizer':
            node = Node(
            package = package_name,
            namespace = '',
            executable = executable_name[i] + '.py',
            name = executable_name[i],
            parameters=[
            { 'frequency': 100.0 },
            { 'L_Base_F1': 0.2 },
            { 'L_F2_F3': 0.25 },
            { 'L_F3_Fe': 0.28 }
            ]
        )
        elif executable_name[i] == 'kinematics':
            node = Node(
            package = package_name,
            namespace = '',
            executable = executable_name[i] + '.py',
            name = executable_name[i],
            parameters=[
            { 'frequency': 100.0 },
            { 'Kp': 5.0 }
            ]
        )
        elif executable_name[i] == 'robot_controller':
            node = Node(
            package = package_name,
            namespace = '',
            executable = executable_name[i] + '.py',
            name = executable_name[i],
            parameters=[
            { 'frequency': 100.0 }
            ]
        )
        elif executable_name[i] == 'robot_scheduler':
            node = Node(
            package = package_name,
            namespace = '',
            executable = executable_name[i] + '.py',
            name = executable_name[i],
            parameters=[
            { 'frequency': 100.0 },
            { 'L_Base_F1': 0.2 },
            { 'L_F2_F3': 0.25 },
            { 'L_F3_Fe': 0.28 }
            ]
        )
        else:
            node = Node(
            package = package_name,
            namespace = '',
            executable = executable_name[i] + '.py',
            name = executable_name[i],
        )
        launch_description.add_action(node)
    
    return launch_description