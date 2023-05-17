# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The launcher contain the information to create the problem istance and the
# information to activate the different nodes
def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_problem5_fluents')
    
    # Create a namespace
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    # Create a launch description 
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        # Define where the model file is located and which namespace to use
        launch_arguments={
          'model_file': example_dir + '/pddl/problem5_domain.pddl',
          'namespace': namespace
          }.items()) 
                
    
    fill_box_cmd = Node(
        package='plansys2_problem5_fluents',
        executable='fill_box_node',
        name='fill_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    load_carrier_cmd = Node(
        package='plansys2_problem5_fluents',
        executable='load_carrier_node',
        name='load_carrier_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    move_carrier_with_box_cmd = Node(
        package='plansys2_problem5_fluents',
        executable='move_carrier_with_box_node',
        name='move_carrier_with_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    unload_carrier_cmd = Node(
        package='plansys2_problem5_fluents',
        executable='unload_carrier_node',
        name='unload_carrier_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    deliver_food_cmd = Node(
        package='plansys2_problem5_fluents',
        executable='deliver_food_node',
        name='deliver_food_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    deliver_medicine_cmd = Node(
        package='plansys2_problem5_fluents',
        executable='deliver_medicine_node',
        name='deliver_medicine_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    deliver_tool_cmd = Node(
        package='plansys2_problem5_fluents',
        executable='deliver_tool_node',
        name='deliver_tool_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    move_carrier_without_box_cmd = Node(
        package='plansys2_problem5_fluents',
        executable='move_carrier_without_box_node',
        name='move_carrier_without_box_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    # Create an istance of the launch description and populate it
    ld = LaunchDescription()

    # Add the declare namespace defined above
    ld.add_action(declare_namespace_cmd)

    # Add the launch options defined above
    ld.add_action(plansys2_cmd)

    ld.add_action(fill_box_cmd)
    ld.add_action(load_carrier_cmd)
    ld.add_action(move_carrier_with_box_cmd)
    ld.add_action(unload_carrier_cmd)
    ld.add_action(deliver_food_cmd)
    ld.add_action(deliver_medicine_cmd)
    ld.add_action(deliver_tool_cmd)
    ld.add_action(move_carrier_without_box_cmd)

    # Return the launch descriptor
    return ld
