import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

from pathlib import Path

project_path = Path(os.path.dirname(__file__))
project_path = Path("/home/strideonr/workspaces/SLAM")

right_camera_config_path = project_path / "BlankCameraCalibration" / "right_camera.yaml"
left_camera_config_path = project_path / "BlankCameraCalibration" / "left_camera.yaml"
camera_config_path = project_path / "BlankCameraCalibration" / "usb_cam_cal_params.yaml"


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
    
    calibrator_node = Node(
        package = 'camera_calibration',
        executable = 'cameracalibrator',
        parameters = [camera_config_path],
        output = 'screen',
        remappings=[('right', '/stereo/right/image_raw'),
                    ('left', '/stereo/left/image_raw'),
                    ('right_camera', '/stereo/right'),
                    ('left_camera', '/stereo/left')
                    
            ]
        
    )
    
    return launch.LaunchDescription([
        usb_cam_right,
        usb_cam_left,
        #calibrator_node
    ])
