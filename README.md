# Ros Mujoco

This package provides a set of plugins for Mujoco to enable communication with ROS. Mujoco is automatically installed with CMake and does not require a separate installation.
The plugins are designed to be added to the MJCF file, allowing for easy integration with ROS topics and services.

This repository is a fork of [MujocoRosUtils](https://github.com/isri-aist/MujocoRosUtils).


## Plugins

The ROS2 plugins for Mujoco share a singleton instance of a `RosContext` object, which grants them access to a node and methods for _spinning_ the node. 
The `RosContext` creates a node with the name `mujoco` when whose lifetime depends on whether any ROS2 plugin is loaded. The `RosContext` does not use
a separate thread for spinning, but instead expects the individual plugins to call the `spinUntilComplete` method if they rely on the node to be spun. Notably,
`spinUntilComplete` is called with the current simulation time, so it only spins _once_ per simulation step.

### RosMujoco::ClockPublisher
Plugin to publish clock topic.

All of the following attributes are optional.
- `topic_name`: Topic name of clock. (Default is `/clock`)
- `publish_rate`: Publish rate. (Default is 100.0 [Hz])

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="RosMujoco::ClockPublisher"/>
</extension>
<worldbody>
  <plugin plugin="RosMujoco::ClockPublisher">
    <config key="topic_name" value="/clock"/>
    <config key="publish_rate" value="100"/>
  </plugin>
</worldbody>
```
This plugin must be registered in worldbody.

### RosMujoco::PosePublisher
Plugin to publish topics or broadcast TF of pose and velocity of the body.

All of the following attributes are optional.
- `frame_id`: Frame ID of topics header or TF parent. (Default is `map`)
- `pose_topic_name`: Topic name of pose. (Default is `mujoco/<body name>/pose`)
- `vel_topic_name`: Topic name of velocity. (Default is `mujoco/<body name>/vel`)
- `publish_rate`: Publish rate. (Default is 30.0 [Hz])
- `output_tf`: Whether to broadcast TF. (Default is `false`)
- `tf_child_frame_id`: Child frame ID for TF. Used only when `output_tf` is `true`. (Default is `<body name>`)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="RosMujoco::PosePublisher"/>
</extension>
<sensor>
  <plugin name="pose_publisher" plugin="RosMujoco::PosePublisher" objtype="xbody" objname="object">
    <config key="frame_id" value="map"/>
    <config key="pose_topic_name" value="/pose"/>
    <config key="vel_topic_name" value="/vel"/>
    <config key="publish_rate" value="30"/>
    <config key="output_tf" value="false"/>
    <config key="tf_child_frame_id" value="object"/>
  </plugin>
</sensor>
```
The `objtype` attribute must be `xbody`.

### RosMujoco::ExternalForce
Plugin to apply external force to the body.

All of the following attributes are optional.
- `topic_name`: Topic name of external force. (Default is `/external_force`)
- `vis_scale`: Arrow length scale. (Default is 0.1)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="RosMujoco::ExternalForce"/>
</extension>
<worldbody>
  <body name="object" pos="0 0 1">
    <freejoint/>
    <geom type="box" size="0.2 0.2 0.05" mass="0.1" rgba="0.5 0.5 0.5 0.3"/>
    <plugin plugin="RosMujoco::ExternalForce">
      <config key="topic_name" value="/external_force"/>
      <config key="vis_scale" value="0.1"/>
    </plugin>
  </body>
</worldbody>
```

### RosMujoco::ImagePublisher
Plugin to publish topics of color and depth images.

All of the following attributes are optional.
- `frame_id`: Frame ID of topics header or TF parent. (Default is `<camera name>`)
- `color_topic_name`: Topic name of color image. (Default is `mujoco/<camera name>/color`)
- `depth_topic_name`: Topic name of depth image. (Default is `mujoco/<camera name>/depth`)
- `info_topic_name`: Topic name of camera information. (Default is `mujoco/<camera name>/camera_info`)
- `height`: Image height. (Default is 240)
- `width`: Image width. (Default is 320)
- `publish_rate`: Publish rate. (Default is 30.0 [Hz])

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="RosMujoco::ImagePublisher"/>
</extension>
<sensor>
  <plugin name="image_publisher" plugin="RosMujoco::ImagePublisher" objtype="camera" objname="camera">
    <config key="frame_id" value="camera"/>
    <config key="color_topic_name" value="/image/color"/>
    <config key="depth_topic_name" value="/image/depth"/>
    <config key="info_topic_name" value="/image/camera_info"/>
    <config key="height" value="240"/>
    <config key="width" value="320"/>
    <config key="publish_rate" value="30"/>
  </plugin>
</sensor>
```
The `objtype` attribute must be `camera`.

### RosMujoco::ActuatorCommand
Plugin to send a command to an actuator via ROS topic.

The following attributes are required.
- `actuator_name`: Actuator name to which the command is sent.
- `topic_name`: Topic name of actuator command. (Default is `mujoco/<actuator name>`)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="RosMujoco::ActuatorCommand"/>
</extension>
<actuator>
  <position name="camera_pan" joint="camera_pan"/>
  <plugin plugin="RosMujoco::ActuatorCommand" joint="camera_pan">
    <config key="actuator_name" value="camera_pan"/>
    <config key="topic_name" value="/camera_pan"/>
  </plugin>
</actuator>
```
In the `plugin` element, you need to specify the `joint`, `body`, etc. of the actuator to be controlled.
This information is not used in the plugin, but is necessary to avoid errors in MJCF parsing.

The plugin itself is also added to the list of actuators, but it is a dummy actuator. The unwanted increase in the number of actuators (which also increases the dimension of `d->ctrl`) is a problem that should be solved in the future.

### RosMujoco::SensorPublisher
Plugin to publish sensor data.

The following attributes are required.
- `sensor_name`: Name of sensor whose data is to be published.

The following attributes are optional.
- `frame_id`: Frame ID of message header. (Default is `map`)
- `topic_name`: Topic name. (Default is `mujoco/<sensor name>`)
- `publish_rate`: Publish rate. (Default is 30.0 [Hz])

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="RosMujoco::SensorPublisher"/>
</extension>
<sensor>
  <rangefinder name="box_rangefinder" site="object_center"/>
  <plugin name="sensor_publisher_scalar" plugin="RosMujoco::SensorPublisher" objtype="xbody" objname="object">
    <config key="sensor_name" value="box_rangefinder"/>
    <config key="frame_id" value="map"/>
    <config key="topic_name" value="/box_rangefinder"/>
    <config key="publish_rate" value="30"/>
  </plugin>
</sensor>
```
In the `plugin` element, you need to specify the `objtype` and `objname`.
This information is not used in the plugin, but is necessary to avoid errors in MJCF parsing.


## Development

The development environment is integrated with VSCode. To use it simply open VSCode and run the task to open the workspace in the dev-container.


## TODO

The launch file relies on the 'simulation' exectuable that MuJoCo builds by default. That only supports launching with a GUI.
To support headless simulation, we need to create a custom executable that supports the same arguments for plugins.