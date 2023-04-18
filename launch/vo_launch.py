from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = False
    logger = LaunchConfiguration("log_level")
    return LaunchDescription([
        # ExecuteProcess(cmd=['ros2', 'bag', 'record',
        #    '/camera/color/image_raw',
        #    '/camera/color/camera_info',
        #    '/camera/depth/color/points',
        # ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('realsense2_camera') ,'/launch/rs_launch.py']),
            launch_arguments={
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
                # 'depth_module.profile': '480,270,60',
                # 'rgb_camera.profile': '424,240,60',
        		'pointcloud.ordered_pc': 'true',
                'rgb_camera.enable_auto_exposure': 'false',
                'depth.enable_auto_exposure': 'false',
                # 'depth_module.hdr_enable': 'true',
                # 'enable_sync': 'true',
                'json_file_path': [get_package_share_directory('vo_nist') ,'/rs_vio.json'],
                }.items(),
        ),
        Node(
            package='vo_nist',
            executable='odom',
        ),
    ])
