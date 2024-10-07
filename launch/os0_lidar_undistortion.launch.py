from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_prefix


min_range = 1.0
max_range = 100.0

key_framing = True
key_frame_dist_thr = 1.5
key_frame_rot_thr = 20.0 * 3.14 / 180.0
key_frame_time_thr = 1.0


def generate_launch_description():
    rviz_file = PathJoinSubstitution(
           [FindPackageShare("lice"), "cfg", "rviz_config.rviz"])
    return LaunchDescription([
        Node(
            package='lice', 
            executable='scan_maker', 
            name='scan_maker',
            remappings=[
                ('/lidar_raw_points', '/os_cloud_node/points'),
                ],
            parameters=[
                {'scan_period': 0.16},
                {'min_range': float(min_range)},
                {'max_range': float(max_range)},
            ],
            output='screen',
        ),
        Node(
            package='lice', 
            executable='lidar_feature_detection', 
            name='lidar_feature_detection',
            parameters=[
                {'min_dist': 1.0},
                {'max_dist': 100.0},
                {'max_planar_pts': 1000}
            ],
            output='screen',
        ),
        Node(
            package='lice', 
            executable='lidar_scan_odometry', 
            name='lidar_scan_odometry',
            remappings=[
                ('/imu/acc', '/os_cloud_node/imu'),
                ('/imu/gyr', '/os_cloud_node/imu')
            ],
            parameters=[
                {"nb_scans_per_submap": 3},
                {"max_feature_dist": 1.0},
                {"state_freq": 1000.0},
                {"publish_all_scans": False},

                # Adapting IMU measurements for some weird IMUs
                {"acc_in_m_per_s2": True},
                {"invert_imu": False},

                # Calibration
                {"calib_px": 0.014},
                {"calib_py": -0.012},
                {"calib_pz": -0.015},
                {"calib_rx": 0.0},
                {"calib_ry": 0.0},
                {"calib_rz": 0.0},

                {"dynamic_filtering": True},
                # Parameters for the dynamic filtering
                {"dynamic_filtering_threshold": 0.4},
                {"dynamic_filtering_voxel_size": 0.15},
                {"key_framing": key_framing},
                {"key_frame_dist_thr": key_frame_dist_thr},
                {"key_frame_rot_thr": key_frame_rot_thr},
                {"key_frame_time_thr": key_frame_time_thr},
            ],
            output='screen',
        ),
        Node(
            package='rviz2', 
            executable='rviz2', 
            name='rviz2',
            output='screen',
            arguments=['-d' , rviz_file],
        )
    ])
