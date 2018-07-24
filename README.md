# Wanderer is running errands ...

## SLAM with [rtapmap_ros](http://wiki.ros.org/rtabmap_ros) package
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/wKk3F-CmCj8/0.jpg)](https://youtu.be/wKk3F-CmCj8)

#### Bring up wanderer
`$ roslaunch ca_driver create_2.launch`

#### Enable joypad control
`$ roslaunch ca_tools joy_teleop.launch [joy_config:=log710]`

#### Bring up realsense
`$ roslaunch realsense2_camera rs_camera.launch align_depth:=true`

#### Mapping and localization with rtabmap
> Note: If SLAM lost, reset odom by the service `$ rosservice call /rtabmap/reset_odom`
- To create map from void
`
$ roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info
`
Map will be stored as ~/.ros/rtabmap.db
- To update current map, do not use arg:`rtabmap_args:="--delete_db_on_start"`
`$ roslaunch rtabmap_ros rtabmap.launch depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info`
- To localize without mapping
`$ roslaunch rtabmap_ros rtabmap.launch depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info localization:=true`
