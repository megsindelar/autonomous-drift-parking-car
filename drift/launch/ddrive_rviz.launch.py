from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_path = get_package_share_path('drift')
    default_model_path = urdf_path / 'ddrive.urdf.xacro'
    default_rviz_config_path = urdf_path / 'ddrive_urdf.rviz'
    view_only_rviz_config_path = urdf_path / 'view_only_ddrive_urdf.rviz'

    view_arg = DeclareLaunchArgument(
        name='view_only',
        default_value='true',
        choices=[
            'true',
            'false'],
        description='Flag to decide how to control joint_states')
    # jsp_arg = DeclareLaunchArgument(name='jsp', default_value='true',
    #                                 choices=['true', 'false'],
    # description='Flag to decide how to control joint_states')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(
        default_model_path), description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(
        name='ddrive_urdf',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')

    view_only_rviz_arg = DeclareLaunchArgument(
        name='view_only_ddrive_urdf',
        default_value=str(view_only_rviz_config_path),
        description='Absolute path to rviz config file')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=LaunchConfigurationEquals('view_only', 'false')
    # )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=LaunchConfigurationEquals('view_only', 'true'),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('ddrive_urdf')],
        condition=LaunchConfigurationEquals('view_only', 'false')
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('view_only_ddrive_urdf')],
        condition=LaunchConfigurationEquals('view_only', 'true')
    )

    return LaunchDescription([
        view_arg,
        # jsp_arg,
        model_arg,
        rviz_arg,
        view_only_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        # joint_state_publisher_node,
        rviz_node,
        rviz2_node
    ])
