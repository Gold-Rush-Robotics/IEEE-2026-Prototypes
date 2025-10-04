import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    servo_params = os.path.join(
        get_package_share_directory('grr_bringup'),
        'config',
        'servo_params.yaml'
    )
    
    roboclaws = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("osr_bringup"),
            "launch",
            "osr_launch.py")]))
    
    motor_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('grr_bringup'),
            'launch',
            'motor_controllers.launch.py'
        )])
    )
    
    drive_train = Node(
        package='grr_python_controllers',
        executable='drivetrain',
        name='drivetrain',
        output='screen',
        emulate_tty=True,
        namespace='grr'
    )
    
    robot = Node(
        package='grr_hardware_python',
        executable='robot',
        name='robot',
        output='screen',
        emulate_tty=True,
        namespace='grr',
        parameters=[servo_params]
    )
    
    gui = Node(
        package='grr_guis',
        executable='gui',
        name='gui',
        output='screen',
        emulate_tty=True,
        namespace='grr'
    )
    
    return LaunchDescription([
        roboclaws,
        motor_controllers,
        robot,
        drive_train,
        gui
    ])
    
    
