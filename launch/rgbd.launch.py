from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_prefix


min_range = 0.1
max_range = 1.5

key_framing = False


def generate_launch_description():
    rviz_file = PathJoinSubstitution(
           [FindPackageShare("ffastllamaa"), "cfg", "rviz_config.rviz"])
    return LaunchDescription([
        Node(
            package='ffastllamaa', 
            executable='gp_map', 
            name='gp_map',
            remappings=[
                ('/points_input', '/camera/depth/color/points'),
                ],
            parameters=[
                {"point_cloud_internal_type": False},
                {"voxel_size": 0.01},
                {"neighbourhood_size": 3},
                {"register": True},
                {"register_with_approximate_field": True},
                {"voxel_size_factor_for_registration": 5.0},
                {"max_num_pts_for_registration": 8000},
                {"use_temporal_weights": False}, # If true, registration weight are 10 times bigger for voxels associated to the older scans than for the newer ones
                {"with_prior": False},
                {"map_publish_period": 0.5},
                {"key_framing": key_framing},

                # Free space carving (<= 0.0 to disable it)
                {"min_range": min_range},
                {"max_range": max_range},
                {"free_space_carving_radius": float(-20)},

                # Path to where the map will be saved
                {"map_path": get_package_prefix('ffastllamaa') + "/share/ffastllamaa/maps/"},
                {"output_normals": True},
                {"output_mesh": True},
                {"meshing_point_per_node": 2.0}, # Min number of points per node (float) for the meshing 1.5 and above, the higher the smoother the mesh
                {"poisson_weighted": False}

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
