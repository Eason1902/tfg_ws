from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    world_path = "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_complex.world"
    robot_path = "/home/yilun/tfg_ws/src/tfg_robot/urdf/tfg_robot.urdf"

    return LaunchDescription([

        # 启动 Gazebo
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
                 f'gz_args:={world_path}'],
            output='screen'
        ),

        # spawn 机器人
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', robot_path,
                '-name', 'tfg_robot',
                '-x', '-10',
                '-y', '-10',
                '-z', '0.2'
            ],
            output='screen'
        )
    ])