import launch
import launch_ros
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'nao_ip',
            default_value="127.0.0.1",
            description='Ip address of the robot'),
        launch.actions.DeclareLaunchArgument(
            'nao_port',
            default_value="9559",
            description='Port to be used for the connection'),
        launch.actions.DeclareLaunchArgument(
            'username',
            default_value="nao",
            description='Username for the connection'),
        launch.actions.DeclareLaunchArgument(
            'password',
            default_value="no_password",
            description='Password for the connection'),
        launch.actions.DeclareLaunchArgument(
            'network_interface',
            default_value="eth0",
            description='Network interface to be used'),
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value="naoqi_driver",
            description='Name of the namespace to be used'),
        launch_ros.actions.Node(
            package='naoqi_driver',
            executable='naoqi_driver_node',
            parameters=[{
                'nao_ip': launch.substitutions.LaunchConfiguration('nao_ip'),
                'nao_port': launch.substitutions.LaunchConfiguration('nao_port'),
                'password': launch.substitutions.LaunchConfiguration('password'),
                'network_interface': launch.substitutions.LaunchConfiguration('network_interface')}],
            output="screen")
    ])