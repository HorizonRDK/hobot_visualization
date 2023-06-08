import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    # visualization node 示例节点pkg
    visualization_node = Node(
        package='hobot_visualization',
        executable='hobot_visualization',
        output='screen',
        parameters=[
            {"msg_pub_topic_name": "/hobot_visualization"},
            {"smart_msg_sub_topic_name": "/hobot_dnn_detection"},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    dnn_node_example = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dnn_node_example'),
                'launch/dnn_node_example.launch.py')),
        launch_arguments={
            'config_file': 'config/fcosworkconfig.json',
            'msg_pub_topic_name': '/hobot_dnn_detection',
            'image_width': str(480),
            'image_height': str(272)
        }.items()
    )

    # jpeg图片编码&发布pkg
    jpeg_compressed_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
            {"channel": 1},
            {"in_mode": "shared_mem"},
            {"in_format": "nv12"},
            {"out_mode": "ros"},
            {"out_format": "jpeg-compressed"},
            {"sub_topic": "/hbmem_img"},
            {"pub_topic": "/image_raw/compressed"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    return LaunchDescription([
        # dnn_node_example推理示例
        dnn_node_example,
        # jpeg图片编码&发布pkg
        jpeg_compressed_codec_node,
        # visualization node 示例节点pkg
        visualization_node,
    ])