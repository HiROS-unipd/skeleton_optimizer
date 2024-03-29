import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    calib_cfg = os.path.join(
        get_package_share_directory('hiros_skeleton_optimizer'),
        'config',
        'calib_test.yaml'
    )

    node = Node(
        package='hiros_skeleton_optimizer',
        executable='hiros_skeleton_optimizer',
        name='skeleton_optimizer',
        namespace='hiros',
        output='screen',
        parameters=[
            {'input_topic': '/input/topic'},
            {'output_topic': '/output/topic'},
            {'number_of_frames_for_calibration': 100},
            {'max_calibration_coefficient_of_variation': 0.2},
            {'outlier_threshold': 0.1},
            {'export_calibration': False},
            {'load_calibration': False},
            {'calibration_file': '/path/to/calibration/file'},
            calib_cfg,
        ]
    )

    ld.add_action(node)
    return ld
