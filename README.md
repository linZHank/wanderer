# Wanderer is running errands ...

## Get Wanderer ready for work

#### Bring up Create2 base
Make sure you connet to wifi "w1010nderer" <br/>
`$ ssh nvidia@192.168.1.3` <br/>
`$ roslaunch ca_driver create_2.launch`

#### (Optional)Remote control
**joypad\:** `$ roslaunch ca_tools joy_teleop.launch [joy_config:=log710]` <br/>
**keyboard\:** `$ rosrun teleop_twist_wasd teleop_twist_wasd.py`

#### Bring up realsense
`$ roslaunch realsense2_camera rs_camera.launch align_depth:=true`

#### SLAM with  [rtapmap_ros](http://wiki.ros.org/rtabmap_ros) package
> Note: If SLAM lost, reset odom by the service `$ rosservice call /rtabmap/reset_odom`
- **To create map from void**
```
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info
```
Map will be stored as ~/.ros/rtabmap.db
- **To update current map**, do not use arg:`rtabmap_args:="--delete_db_on_start"` <br/>
```
roslaunch rtabmap_ros rtabmap.launch depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info
```
- **To localize without mapping**
```
roslaunch rtabmap_ros rtabmap.launch depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info localization:=true
```
#### Publish TF of camera_link
`$ rosrun wanderer_navigation tf_publisher.py`

#### Take Wanderer back home
We have a simple demonstration code of taking Wanderer to the origin of the map from any other point in the map (any orientation). This demo requires neither obstacles or turn in map. Wanderer just aims at the origin, then moves straight towards it. Wanderer does check its orientation frequently to make sure it is still heading towards the correct target.  
`$ rosrun wanderer_navigation go_home_v2.py`

## Setup Demo
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/wKk3F-CmCj8/0.jpg)](https://youtu.be/wKk3F-CmCj8)

## Create Map (uncut)
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/wZM87L1v570/0.jpg)](https://youtu.be/wZM87L1v570)

## Go Home (uncut)
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/x4rCqfsHLwc/0.jpg)](https://youtu.be/x4rCqfsHLwc)
