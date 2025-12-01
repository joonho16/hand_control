from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 하드웨어 제어 노드 (STM32/Dynamixel 통신)
        Node(
            package='hand_control',
            executable='dxl_driver',  # setup.py entry_points에 등록된 이름
            name='dxl_driver_node',
            output='screen'
        ),
        
        # 2. GUI 노드 (Qt 화면)
        Node(
            package='hand_control',
            executable='hand_gui',    # setup.py entry_points에 등록된 이름
            name='hand_gui_node',
            output='screen'
        )
    ])