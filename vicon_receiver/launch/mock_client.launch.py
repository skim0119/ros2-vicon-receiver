from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    buffer_size = 200
    topic_namespace = "vicon_mock"

    return LaunchDescription(
        [
            Node(
                package="vicon_receiver_mock",
                executable="vicon_client",
                output="screen",
                parameters=[{"buffer_size": buffer_size, "namespace": topic_namespace}],
            )
        ]
    )
