from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
import launch_ros.descriptions
from launch_ros.actions import Node
import launch_ros
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    stretch_core_path = get_package_share_path('stretch_core')

    declare_broadcast_odom_tf_arg = DeclareLaunchArgument(
        'broadcast_odom_tf',
        default_value='False', choices=['True', 'False'],
        description='Whether to broadcast the odom TF'
    )

    declare_fail_out_of_range_goal_arg = DeclareLaunchArgument(
        'fail_out_of_range_goal',
        default_value='False', choices=['True', 'False'],
        description='Whether the motion action servers fail on out-of-range commands'
    )

    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='position', choices=['position', 'navigation', 'trajectory', 'gamepad'],
        description='The mode in which the ROS driver commands the robot'
    )

    declare_controller_arg = DeclareLaunchArgument(
        'calibrated_controller_yaml_file',
        default_value=str(stretch_core_path / 'config' / 'controller_calibration_head.yaml'),
        description='Path to the calibrated controller args file'
    )

    joy_runstop_enabled_arg = DeclareLaunchArgument(
        'joy_runstop_enabled',
        default_value='True', choices=['True', 'False'],
        description='Whether the joystick runstop is enabled'
    )

    split_joint_trajectory_controller_arg = DeclareLaunchArgument(
        'split_joint_trajectory_controller',
        default_value='False', choices=['True', 'False'],
        description='Whether to use the split_joint_trajectory classes (Head and Body separate)'
    )

    # joy_node = Node(package='joy',
    #                 executable='joy_node',
    #                 output='screen',
    #                 condition=LaunchConfigurationEquals('joy_runstop_enabled', TextSubstitution(text='True')))

    robot_description_content = launch_ros.parameter_descriptions.ParameterValue( Command(['xacro ', str(get_package_share_path('stretch_description') / 'urdf' / 'stretch.urdf')]), value_type=str)

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 output='log',
                                 parameters=[{'source_list': ['/stretch/joint_states']},
                                             {'rate': 30.0}],
                                 arguments=['--ros-args', '--log-level', 'error'],)

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': robot_description_content},
                                             {'publish_frequency': 30.0}],
                                 arguments=['--ros-args', '--log-level', 'error'],)

    stretch_driver_params = [
        {'rate': 30.0,
         'timeout': 0.5,
         'controller_calibration_file': LaunchConfiguration('calibrated_controller_yaml_file'),
         'broadcast_odom_tf': LaunchConfiguration('broadcast_odom_tf'),
         'fail_out_of_range_goal': LaunchConfiguration('fail_out_of_range_goal'),
         'mode': LaunchConfiguration('mode'),
         'joy_runstop_enabled': LaunchConfiguration('joy_runstop_enabled'),
         'split_joint_trajectory_controller': LaunchConfiguration('split_joint_trajectory_controller'),}
    ]
    print(stretch_driver_params)

    stretch_driver = Node(package='stretch_core',
                          executable='stretch_driver',
                          emulate_tty=True,
                          output='screen',
                          remappings=[('cmd_vel', '/stretch/cmd_vel'),
                                      ('joint_states', '/stretch/joint_states')],
                          parameters=stretch_driver_params)

    return LaunchDescription([declare_broadcast_odom_tf_arg,
                              declare_fail_out_of_range_goal_arg,
                              declare_mode_arg,
                              declare_controller_arg,
                              joy_runstop_enabled_arg,
                              split_joint_trajectory_controller_arg,
                              joint_state_publisher,
                              robot_state_publisher,
                            #   joy_node,
                              stretch_driver])
