from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    world_path = "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_simple.world"
    robot_path = "/home/yilun/tfg_ws/src/tfg_robot/models/turtlebot3_burger/model.sdf"
    model_path = "/home/yilun/tfg_ws/src/tfg_robot/models"

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_path
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
                f'gz_args:=-r {world_path}'
            ],
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-file', robot_path,
                        '-name', 'tb3',
                        '-x', '-9',
                        '-y', '-9',
                        '-z', '0.1'
                    ],
                    output='screen'
                )
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            ],
            output='screen'
        ),

        Node(
            package='tfg_robot',
            executable='odom_tf_publisher',
            output='screen'
        ),
    ])