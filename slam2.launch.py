import os
import launch
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml

def generate_launch_description():

    right_usb_params_file = '/home/orl/Documents/CameraCalibration/right_camera.yaml'
    left_usb_params_file = '/home/orl/Documents/CameraCalibration/left_camera.yaml'
    
    with open(right_usb_params_file, 'r') as file:
        right_usb_params = yaml.safe_load(file)

    with open(left_usb_params_file, 'r') as file:
        left_usb_params = yaml.safe_load(file)    

    usb_cam_right = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='stereo/right',
        output='screen',
        parameters=[right_usb_params]
    )

    usb_cam_left = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='stereo/left',
        output='screen',
        parameters=[left_usb_params]
    )

    camera_base_tf = Node(
        package = 'tf2_ros',
        executable='static_transform_publisher',
        output= 'screen',
        arguments = [
            "--x", "1",
            "--y", "1",
            "--z", "1",
            "--roll", "1",
            "--pitch", "1",
            "--yaw", "1",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link"

        ]
    )

    right_base_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "camera_link",
                "--child-frame-id", "right_camera_link"
                ]
        )  
    
    left_base_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "2",
                "--y", "2",
                "--z", "2",
                "--roll", "2",
                "--pitch", "2",
                "--yaw", "2",
                "--frame-id", "camera_link",
                "--child-frame-id", "left_camera_link"
                ]
        )
    
    
    
    
    return launch.LaunchDescription([
        usb_cam_right,
        usb_cam_left,
        camera_base_tf,
        left_base_tf,
        right_base_tf
    ])
