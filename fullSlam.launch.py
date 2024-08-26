import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

from pathlib import Path

project_path = Path(os.path.dirname(__file__))

right_camera_config_path = project_path / "CameraCalibration" / "right_camera.yaml"
left_camera_config_path = project_path / "CameraCalibration" / "left_camera.yaml"
image_proc_rectify_config_path = project_path / "ImageProcRectify" / "image_proc_rectify_parameters.yaml"

####################################################
###################### TO DO  ######################
#################################################### 

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
                "--x", "0",
                "--y", ".040",
                "--z", "0.029",                
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "camera_link",
                "--child-frame-id", "camera2"
            ]
    )
    
    #Left camera to base link
    left_base_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0",
                "--y", "-0.040",
                "--z", "0.029",   
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "camera_link",
                "--child-frame-id", "camera1"
            ]
    )
    
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            
        
        'rectified_images': True,

        'enable_imu_fusion': False,
        'debug_imu_mode': False,
        'enable_debug_mode': False,
        'debug_dump_path': 'debug',

        'enable_image_denoising': True,
        
        'enable_slam_visualization': False, # visualization may affects the performance
        'enable_landmarks_view': False, # visualization may affects the performance
        'enable_observations_view': False, # visualization may affects the performance

        'enable_localization_n_mapping': True,

        'image_jitter_threshold_ms': 35.00, # for 30 FPS
        'sync_matching_threshold_ms': 10.0, # desync in ms between different cams

        'num_cameras': 2, # 2 cams within one stereo camera
        'base_frame': 'camera_link',
        'verbocity': 3,
        
        'image_buffer_size': 200,
    
        }],

        remappings=[('visual_slam/image_0', '/stereo/left/image_raw'),
                    ('visual_slam/camera_info_0', '/stereo/left/camera_info'),
                    ('visual_slam/image_1', '/stereo/right/image_raw'),
                    ('visual_slam/camera_info_1', '/stereo/right/camera_info')
            ]
    )

    ####################################################
    ################# IMAGE PROC NODES #################
    #################################################### 


        #Right Camera Processing
    right_camera_proc = ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace= right_namespace,
            parameters=[image_proc_rectify_config_path]
        )
    #Left Camera Processing
    left_camera_proc = ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace= left_namespace,
            parameters=[image_proc_rectify_config_path]
        )

        
        


    # #Container Generation
    # arg_container = DeclareLaunchArgument(
    #     name = 'container', default_value='',
    #     description=(
    #         'Name of an existing node container to load launched nodes into. '
    #         'If unset, a new container will be created'
    #     )
    # )

    # #If an existing container is not provided, start a container and load nodes into it
    # image_processing_container = ComposableNodeContainer(
    #     condition=LaunchConfigurationEquals('container', ''),
    #     name='image_proc_container',
    #     namespace="image_proc_container",
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=slam_node,
    #     output='screen'
    # )

    # # If an existing container name is provided, load composable nodes into it
    # # This will block until a container with the provided name is available and nodes are loaded
    # load_composable_nodes = LoadComposableNodes(
    #     condition=LaunchConfigurationNotEquals('container', ''),
    #     composable_node_descriptions=composable_nodes,
    #     target_container=LaunchConfiguration('container'),
    # )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )
    
    image_proc_launch_container = ComposableNodeContainer(
        name='image_proc_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[left_camera_proc, right_camera_proc],
        output='screen'
    )
    
    
    return launch.LaunchDescription([
        usb_cam_right,
        usb_cam_left,
        camera_base_tf,
        left_base_tf,
        right_base_tf,
        # arg_container,
        visual_slam_launch_container,
        #image_proc_launch_container
        # load_composable_nodes
    ])