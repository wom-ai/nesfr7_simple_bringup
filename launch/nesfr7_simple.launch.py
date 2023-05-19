import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.substitutions import FindPackageShare
import os

os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'


def generate_launch_description():
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='xbox_joy_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        namespace=LaunchConfiguration('namespace'),
        remappings=[
            ('joy', 'xbox_joy'),
            ('joy/set_feedback', 'xbox_joy/set_feedback'),
            ]
    )
    joy_switch_node = launch_ros.actions.Node(
        package='nesfr7_simple_bringup',
        namespace=LaunchConfiguration('namespace'),
        executable='joy_switch.py',
        output='both')

    teleop_node = launch_ros.actions.Node(
        package='nesfr_teleop',
        executable='nesfr_teleop_node',
        output='screen',
        name='nesfr_teleop_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        namespace=LaunchConfiguration('namespace')
    )
    bridge_node = launch_ros.actions.Node(
        package='nesfr_bridge',
        executable='nesfr_ros_bridge',
        name='nesfr_ros_bridge',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        namespace=LaunchConfiguration('namespace')
    )
    nesfr7_arm_common_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nesfr_arm_bringup'), 'launch',
                    'nesfr7_arm_common.launch.py'
                    ])
                ]),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace')
                }.items()
            )
    nesfr_system_exec = ExecuteProcess(
            cmd=[[
                    FindExecutable(name='nesfr_system'),
                    ' --config /usr/local/share/nesfr_system/config/main.config',
                ]],
            name='nesfr_system',
            shell=True,
            output='both'
            )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='namespace', default_value='',
                                             description='namespace'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                             description='Flag to enable use_sim_time'),
        nesfr_system_exec,
        joy_node,
        joy_switch_node,
        teleop_node,
        TimerAction(period=1.0, actions=[bridge_node]),
        TimerAction(period=1.0, actions=[nesfr7_arm_common_launch]),
    ])
