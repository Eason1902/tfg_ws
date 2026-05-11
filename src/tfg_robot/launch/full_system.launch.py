from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    world_path = "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_complex.world"
    robot_path = "/home/yilun/tfg_ws/src/tfg_robot/models/tfg_robot.sdf"

    return LaunchDescription([

        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
                 f'gz_args:={world_path}'],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', robot_path,
                '-name', 'tfg_robot',
                '-x', '-10',
                '-y', '-10',
                '-z', '0.3'
            ],
            output='screen'
        ),

        # TF 发布
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(robot_path).read()}]
        )
    ])
