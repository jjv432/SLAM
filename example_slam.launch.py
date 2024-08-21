import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            
        
        'rectified_images': False,

        'enable_imu_fusion': False,
        'debug_imu_mode': False,
        'enable_debug_mode': False,

        'enable_slam_visualization': True, # visualization may affects the performance
        'enable_landmarks_view': True, # visualization may affects the performance
        'enable_observations_view': False, # visualization may affects the performance

        'enable_localization_n_mapping': False,

        'image_jitter_threshold_ms': 40.00, # for 30 FPS
        'sync_matching_threshold_ms': 10.0, # desync in ms between different cams

        'num_cameras': 2, # 2 cams within one stereo camera
        'base_frame': 'camera_link',
        'verbocity': 3,


            # 'camera_optical_frames': [
            #     'left_camera_link',
            #     'right_camera_link',
            # ],
        }],

        remappings=[('visual_slam/image_0', '/stereo/left/image_mono'),
                    ('visual_slam/camera_info_0', '/stereo/left/camera_info'),
                    ('visual_slam/image_1', '/stereo/right/image_mono'),
                    ('visual_slam/camera_info_1', '/stereo/right/camera_info')
            ]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([visual_slam_launch_container])
