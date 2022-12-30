# ros_components_description
URDF models of sensors and other components offered alongside Husarion robots

## Including sensor

First build the package by running:
``` bash
git clone -b ros1 https://github.com/husarion/ros_components_description.git
# in case the package will be used within simulation
export HUSARION_ROS_BUILD=simulation
rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release
```

To include the sensor, use the following code:
``` xml
<!-- include file with definition of xacro macro of sensor -->
<xacro:include filename="$(find ros_components_description)/urdf/slamtec_rplidar_s1.urdf.xacro" ns="lidar" />

<!-- evaluate the macro and place the sensor on robot -->
<xacro:lidar.slamtec_rplidar_s1
  parent_link="cover_link"
  xyz="0.0 0.0 0.0"
  rpy="0.0 0.0 0.0"
  use_gpu="true"
  simulation_engine="gazebo-classic" />
```

A list of parameters can be found here:
- `parent_link` [*string*, default: **None**] parent link to which sensor should be attached.
- `xyz` [*float list*, default: **None**] 3 float values defining translation between base of a sensor and parent link. Values in **m**.
- `rpy` [*float list*, default: **None**] 3 float values define rotation between parent link and base of a sensor. Values in **rad**.
- `tf_prefix` [*string*, optional] tf prefix applied before all links created by sensor. If defined, applies `<tf_prefix>_<sensor_name>`. If not defined, leaves `<sensor_name>` intact. Applies also to `frame_id` parameter. 
- `topic` [*string*, default: **same as default of manufacturer**] name of topic at which simulated sensor will publish data.
- `frame_id` [*string*, default: **same as default of manufacturer**] name of final tf to which sensor will be attached. Should match one from message published by sensor.
- `use_gpu` [*bool*, default: **false**] enable GPU acceleration for sensor. Available only if sensor can be accelerated.
- `simulation_engine` [*string*, default: **gazebo-classic**] selected for which simulation engine plugins should be loaded. Currently the only supported value is **gazebo-classic** standing for [Gazebo Classic](https://classic.gazebosim.org/).

Some sensors can define their specific parameters. Refer to their definition for more info.