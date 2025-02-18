from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.actions

from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    rviz = LaunchConfiguration('rviz', default='false')
    rviz_use_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RVIZ'
    )
    
    config_file = PathJoinSubstitution([get_package_share_directory('faster_lio'), 'config', 'ouster128.yaml'])
    rviz_config = PathJoinSubstitution([get_package_share_directory('faster_lio'), 'rviz_cfg', 'loam_livox.rviz'])
    

    faster_lio_node=Node(
        package='faster_lio',
        executable="run_mapping_online",
        output='screen',
        parameters=[ config_file
                    ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config,
                    '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time' : True}],
        output='screen',
        condition=IfCondition(rviz)
    )

    ld = LaunchDescription([launch_ros.actions.SetParameter(name='use_sim_time', value=True),
                            ])
    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_colorized_envvar)
    ld.add_action(faster_lio_node)
    ld.add_action(rviz_node)

    return ld
