from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    relay_node = Node(
        package="gap_follow",
        executable="relay.py",

        name="my_relay",
        remappings=[
            ("drive_relay", "my_drive_relay"),
            ("drive", "my_drive")
        ]
    )
    
    talker_node = Node(
        package="gap_follow",
        executable="talker.py",

        name="my_talker",
        remappings=[
            ("drive", "my_drive")
        ],
        parameters=[
            {"v": 4.5},
            {"d": 7.25}
        ]
    )
    
    ld.add_action(relay_node)
    ld.add_action(talker_node)
    
    return ld