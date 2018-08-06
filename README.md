# Wanderer is running errands ...

## Demo
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/wKk3F-CmCj8/0.jpg)](https://youtu.be/wKk3F-CmCj8)

#### Bring up wanderer
`$ roslaunch ca_driver create_2.launch`

#### Remote control
**joypad\:** `$ roslaunch ca_tools joy_teleop.launch [joy_config:=log710]` <br/>
**keyboard\:** `$ rosrun teleop_twist_wasd teleop_twist_wasd.py`

#### Bring up realsense
`$ roslaunch realsense2_camera rs_camera.launch align_depth:=true`

#### SLAM with  [rtapmap_ros](http://wiki.ros.org/rtabmap_ros) package
> Note: If SLAM lost, reset odom by the service `$ rosservice call /rtabmap/reset_odom`
- To create map from void
`
$ roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info
`
Map will be stored as ~/.ros/rtabmap.db
- To update current map, do not use arg:`rtabmap_args:="--delete_db_on_start"` <br/>
`$ roslaunch rtabmap_ros rtabmap.launch depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info`
- To localize without mapping
`$ roslaunch rtabmap_ros rtabmap.launch depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info localization:=true`

#### Publish TF of camera_link
`$ rosrun wanderer_navigation tf_publisher.py`

## Create Map (uncut)
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/wZM87L1v570/0.jpg)](https://youtu.be/wZM87L1v570)

## Go Home (uncut)
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/x4rCqfsHLwc/0.jpg)](https://youtu.be/x4rCqfsHLwc)
