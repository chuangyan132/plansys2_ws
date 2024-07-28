'''
@Description: It launch the llm2bt_executor node.
@Maintainer:  Chuang Yan
@Email:       yanchuang1122@gmail.com
@Date:        25.07.2024
'''

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    # Get the launch file directory
    llm2bt_executor_dir = get_package_share_directory('llm2bt_executor')

    # Launch the llm2bt_executor node
    llm2bt_executor_node = Node(
        package='llm2bt_executor',
        executable='llm2bt_executor',
        name='llm2bt_executor',
        output='screen',
        parameters=[
            
        ]
    )

    ld.add_action(llm2bt_executor_node)

    return ld