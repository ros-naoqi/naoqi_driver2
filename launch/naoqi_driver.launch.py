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
            'qi_listen_url',
            default_value="tcp://0.0.0.0:0",
            description='Endpoint to listen for incoming NAOqi connections (for audio)'),
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value="naoqi_driver",
            description='Name of the namespace to be used'),
        launch.actions.DeclareLaunchArgument(
            'emulation_mode',
            default_value="false",
            description='Enable emulation mode (fake NAOqi without real robot)'),
        launch.actions.DeclareLaunchArgument(
            'robot_type',
            default_value="nao",
            description='Robot type for emulation (nao, pepper, or romeo)'),
        launch_ros.actions.Node(
            package='naoqi_driver',
            executable='naoqi_driver_node',
            parameters=[{
                'nao_ip': launch.substitutions.LaunchConfiguration('nao_ip'),
                'nao_port': launch.substitutions.LaunchConfiguration('nao_port'),
                'password': launch.substitutions.LaunchConfiguration('password'),
                'network_interface': launch.substitutions.LaunchConfiguration('network_interface'),
                'qi_listen_url': launch.substitutions.LaunchConfiguration('qi_listen_url'),
                'emulation_mode': launch.substitutions.LaunchConfiguration('emulation_mode'),
                'robot_type': launch.substitutions.LaunchConfiguration('robot_type'),
            }],
            output="screen")
    ])