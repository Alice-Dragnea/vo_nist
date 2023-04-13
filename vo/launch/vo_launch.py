from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess(cmd=['ros2', 'bag', 'record',
        #     '/camera/color/image_raw',
        #     '/camera/color/camera_info',
        #     '/camera/depth/color/points'
        #     ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
            launch_arguments={
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
                'depth_module.profile': '848,480,60',
                'rgb_camera.profile': '848,480,60',
                'pointcloud.ordered_pc': 'true',
                }.items(),
        ),
        # Node(package='vo', executable='run.py', output='screen')
    ])
