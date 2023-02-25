import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    run_rviz_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('drift')),
                '/ddrive_rviz.launch.py']),
        launch_arguments={
            'view_only': 'false'}.items())

    urdf_path = get_package_share_path('drift')
    default_model_path = urdf_path / 'ddrive.urdf.xacro'

    package_share = os.path.join(get_package_share_directory('drift'))
    gazebo_models_path = os.path.join(package_share, 'ddrive_urdf')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    model_arg = DeclareLaunchArgument(name='model', default_value=str(
        default_model_path), description='Absolute path to robot urdf file')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    start_gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory(
            'ros_ign_gazebo')), '/launch/ign_gazebo.launch.py']),
        launch_arguments={'ign_args': [os.path.join(get_package_share_directory(
            'drift')), '/ddrive.world -r']}.items())

    spawn_model = Node(package='ros_gz_sim', executable='create',
                       arguments=['-name', 'Jerry',
                                  '-x', '-4.5',
                                  '-y', '1.0',
                                  '-z', '0.3',
                                  '-topic', '/robot_description'],
                       output='screen')

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='Jerry_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/jerry_odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/world/ddrive_world/model/Jerry/joint_state@' +
            'sensor_msgs/msg/JointState[ignition.msgs.Model',
            'jerry_tf@'
            'tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[
            ('/world/ddrive_world/model/Jerry/joint_state',
             'joint_states'),
            ('jerry_tf',
             '/tf')])

    drift = Node(package='drift', executable='drift')

    return LaunchDescription([
        model_arg,
        start_gazebo,
        run_rviz_launch_file,
        robot_state_publisher_node,
        spawn_model,
        drift,
        bridge
    ])
