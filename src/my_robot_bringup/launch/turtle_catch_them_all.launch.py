from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    spawner_node = Node(
        package="turtle_game",
        executable="turtlesim_spawner"
    )

    turtle1_controller = Node(
        package="turtle_game",
        executable="turtle_controller"    
    )
    

    ld.add_action(turtlesim_node)
    ld.add_action(spawner_node)
    ld.add_action(turtle1_controller)
 
    return ld
