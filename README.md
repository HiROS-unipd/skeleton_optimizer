# Hi-ROS Skeleton Optimizer


## Dependencies
* [Hi-ROS Skeleton Messages](https://gitlab.com/hi-ros/skeleton_msgs)


## Launch files
**default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice


## Usage
This ROS package takes as input a skeleton group and optimizes the markers positions.

```
roslaunch hiros_skeleton_optimizer custom_configuration_example.launch
```

* ## Parameters

  | Parameter                                  | Description                                                  |
  | ------------------------------------------ | ------------------------------------------------------------ |
  | `node_required`                            | Set if the other ROS nodes on the PC should be killed when the driver is killed |
  | `node_name`                                | Node's name                                                  |
  | `input_topic_name`                         | Name of the input topic                                      |
  | `output_topic_name`                        | Name of the topic that will be published containing the optimized skeletons |
  | `number_of_frames_for_calibration`         | Number of frames to acquire for the initial calibration of the link lengths |
  | `max_calibration_coefficient_of_variation` | Maximum coefficient of variation to accept the calibration   |
  | `outlier_threshold`                        | Maximum acceptable link length variation; a higher variation indicates the presence of an outiler |

  
