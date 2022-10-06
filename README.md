# Hi-ROS Skeleton Optimizer

This ROS package takes as input a skeleton group and optimizes the marker positions and link orientations to avoid varying limb lengths.


## Dependencies
* [Hi-ROS Skeleton Messages](https://github.com/hiros-unipd/skeleton_msgs)
* [Hi-ROS Skeleton Tracker](https://github.com/hiros-unipd/skeleton_tracker)


## Parameters
| Parameter                                  | Description                                                                                       |
| ------------------------------------------ | ------------------------------------------------------------------------------------------------- |
| `input_topic`                              | Name of the input topic                                                                           |
| `output_topic`                             | Name of the output topic containing the optimized skeletons                                       |
| `number_of_frames_for_calibration`         | Number of frames to acquire for the initial calibration of the link lengths                       |
| `max_calibration_coefficient_of_variation` | Maximum coefficient of variation to accept the calibration                                        |
| `outlier_threshold`                        | Maximum acceptable link length variation; a higher variation indicates the presence of an outiler |
| `export_calibration`                       | Set if calibration data should be exported to file                                                |
| `load_calibration`                         | Set if calibration data should be loaded from file                                                |
| `calibration_file`                         | Name of the file where calibration data is imported/exported                                      |


## Usage
```
ros2 launch hiros_skeleton_optimizer default.launch.py
```
