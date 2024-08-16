import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


#Don't put commas after any of these!
# PROJECT_PATH = "workspaces/Slam/SLAM"
# RIGHT_CAMERA_CONFIG_PATH = os.path.join(PROJECT_PATH, "CameraCalibration/right_camera.yaml")
# LEFT_CAMERA_CONFIG_PATH = os.path.join(PROJECT_PATH, "CameraCalibration/left_camera.yaml")
# IMAGE_PROC_RECTIFY_CONFIG_PATH = os.path.join(PROJECT_PATH, "ImageProc/image_proc_rectify_parameters.yaml")
# print(RIGHT_CAMERA_CONFIG_PATH)
# print(LEFT_CAMERA_CONFIG_PATH)

from pathlib import Path

project_path = Path("workspaces/Slam/SLAM")

right_camera_config_path = project_path / "CameraCalibration" / "right_camera.yaml"
left_camera_config_path = project_path / "CameraCalibration" / "left_camera.yaml"
image_proc_rectify_config_path = project_path / "ImageProcRectify" / "image_proc_rectify_parameters.yaml"

print(right_camera_config_path)
print(left_camera_config_path)
print(image_proc_rectify_config_path)

####################################################
###################### TO DO  ######################
#################################################### 

#See if the container stuff needs to happen twice or at all


def generate_launch_description():
    
    left_namespace = 'stereo/left',
    right_namespace = 'stereo/right',  


    ####################################################
    ################# USB CAMERA NODES #################
    #################################################### 

    #Right Usb Camera
    usb_cam_right = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace=right_namespace,
        output='screen',
        parameters=[right_camera_config_path],
     
    )

    #Left Usb Camera
    usb_cam_left = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace=left_namespace,
        output='screen',
        parameters=[left_camera_config_path],
      
    )

    ####################################################
    ################# IMAGE PROC NODES #################
    #################################################### 

    #Right Camera Processing
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace= right_namespace,
           
        ),
        ComposableNode(
            package = 'image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_mono_node',
            namespace=right_namespace,
            parameters = [image_proc_rectify_config_path],
            remappings=[
                ('image', 'image_mono'),
                ('image_rect', 'image_rect')
            ],
         

        )    
    ]

    #Left Camera Processing
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace= left_namespace,
       
        ),
        ComposableNode(
            package = 'image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_mono_node',
            namespace=right_namespace,
            parameters = [image_proc_rectify_config_path],
            remappings=[
                ('image', 'image_mono'),
                ('image_rect', 'image_rect')
            ]

        )    
    ]

    #Container Generation
    arg_container = DeclareLaunchArgument(
        name = 'container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created'
        )
    )

      # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_proc_container',
        namespace="image_proc_container",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )

    ####################################################
    ################## TRANSFORM NODES #################
    #################################################### 

    #Camera base link to odom frame
    camera_base_tf = Node(
        package = 'tf2_ros',
        executable='static_transform_publisher',
        output= 'screen',
        arguments = [
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "odom",
            "--child-frame-id", "camera_link"

             ]
        )

    #Right camera to base link
    right_base_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", ".080",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "camera_link",
                "--child-frame-id", "right_camera_link"
                ]
        )
    
    #Left camera to base link
    left_base_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "-.040",
                "--y", ".029",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "camera_link",
                "--child-frame-id", "left_camera_link"
                ]
        )
    
    return launch.LaunchDescription([
        usb_cam_right,
        usb_cam_left,
        camera_base_tf,
        left_base_tf,
        right_base_tf,
        arg_container,
        image_processing_container,
        load_composable_nodes
    ])
