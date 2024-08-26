import os
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from pathlib import Path

project_path = Path(os.path.dirname(__file__))
right_camera_config_path = project_path / "CameraCalibration" / "right_camera.yaml"
left_camera_config_path = project_path / "CameraCalibration" / "left_camera.yaml"

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
    
    disparity_node = Node(
        package = 'stereo_image_proc',
        executable='disparity_node',
        remappings=[
            ('left/camera_info', 'stereo/left/camera_info'),
            ('right/camera_info', 'stereo/right/camera_info'),
            ('left/image_rect', 'stereo/left/image_raw'),
            ('right/image_rect', 'stereo/right/image_raw')
            
        ],
        parameters = [{
            'approximate_sync' : True,
            'queue_size' : 10
        }]
    )
    
    point_node = Node(
        package = 'stereo_image_proc',
        executable='point_cloud_node',
        remappings=[
            ('left/camera_info', 'stereo/left/camera_info'),
            ('right/camera_info', 'stereo/right/camera_info'),
            ('left/image_rect_color', 'stereo/left/image_raw'),
            ('disparity', 'disparity')
            
        ],
        parameters = [{
            'approximate_sync' : True,
            'approximate_sync_tolerance_seconds' : 1.1,
            'stereo_algorithm': 1
        }]
    )
    
    rqt_node_disparity = Node(
        package = 'image_view',
        executable= 'disparity_view',
         remappings=[
            ('image', 'disparity'),
        ],
        
    )
    
    rqt_node_left = Node(
        package = 'image_view',
        executable= 'image_view',
         remappings=[
            ('image', 'stereo/left/image_raw'),
        ],
        
    )
    
    rqt_node_right = Node(
        package = 'image_view',
        executable= 'image_view',
         remappings=[
            ('image', 'stereo/right/image_raw'),
        ],
        
    )
    return launch.LaunchDescription([
        usb_cam_right,
        usb_cam_left,
        disparity_node,
        # point_node,
        rqt_node_left,
        rqt_node_right,
        rqt_node_disparity
        ])