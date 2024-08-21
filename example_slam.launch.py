import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            
        'enable_image_denoising': False,
        'enable_slam_visualization': False,
           'rectified_images': True,
           'enable_imu_fusion': False,
           'image_jitter_threshold_ms': 40.00,
           'base_frame': 'camera_link',
           'enable_slam_visualization': True,
           'enable_landmarks_view': False,
           'enable_observations_view': False,

            'num_cameras': 2, # 2 cams within one stereo camera
            'base_frame': 'camera_link',
            'verbocity': 3,

            'camera_optical_frames': [
                'left_camera_link',
                'right_camera_link',
            ],
        }],

        remappings=[('visual_slam/image_0', '/stereo/left/image_rect'),
                    ('visual_slam/camera_info_0', '/stereo/left/camera_info'),
                    ('visual_slam/image_1', '/stereo/right/image_rect'),
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