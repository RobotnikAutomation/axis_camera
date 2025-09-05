# axis_camera

This package contains the ROS2 nodes for controlling and streaming AXIS PTZ cameras.

To run both of them:

```bash
ros2 launch axis_camera axis_camera.launch.py
```

# Dependencies

## ROS2
- [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/index-msg.html)
- [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/index-msg.html)
- [robotnik_sensors_msgs](https://github.com/RobotnikAutomation/robotnik_interfaces)
- [robotnik_actuators_msgs](https://github.com/RobotnikAutomation/robotnik_interfaces)
- [std_srvs](https://docs.ros2.org/foxy/api/std_srvs/index-msg.html)
- [cv_bridge](https://github.com/ros-perception/vision_opencv)
- [camera_info_manager_py](https://github.com/ros-perception/image_common/tree/jazzy)

## Python
- numpy
- opencv
- requests

---
# 1. axis_ptz_node

The `axis_ptz_node` runs the PTZ control of the camera.

## 1.1. Parameters

* **hostname** (string, default: "192.168.0.185"): IP address of the connected AXIS camera.

* **camera_number** (int, default: 1): Camera number associated to the camera.
* **desired_freq** (float, default: 20.0): Desired frequency (Hz) for the node loop.
* **connection_timeout** (float, default: 5.0): Maximum time (seconds) to wait when establishing a connection.
* **pan.min_value** (float, default: -PI): Minimum pan angle (radians).
* **pan.max_value** (float, default: PI): Maximum pan angle (radians).
* **pan.joint** (string, default: 'axis_pan_joint'): Joint name for pan control.
* **pan.offset** (float, default: 0.0): Offset for pan position (radians).
* **pan.error_pos** (float, default: 0.01): Acceptable error position for pan (radians).
* **pan.invert** (bool, default: False): Invert pan direction if true.
* **tilt.min_value** (float, default: 0.0): Minimum tilt angle (radians).
* **tilt.max_value** (float, default: PI/2): Maximum tilt angle (radians).
* **tilt.joint** (string, default: 'axis_tilt_joint'): Joint name for tilt control.
* **tilt.offset** (float, default: 0.0): Offset for tilt position (radians).
* **tilt.error_pos** (float, default: 0.01): Acceptable error position for tilt (radians).
* **tilt.invert** (bool, default: False): Invert tilt direction if true.
* **zoom.min_value** (float, default: 1.0): Minimum zoom level.
* **zoom.max_value** (float, default: 9999.0): Maximum zoom level.
* **zoom.joint** (string, default: 'axis_zoom_joint'): Joint name for zoom control.
* **zoom.offset** (float, default: 0.0): Offset for zoom position (zoom level).
* **zoom.error_pos** (float, default: 99.0): Acceptable error position for zoom (zoom level).
* **zoom.min_augment** (float, default: 0.0): Minimum zoom augmentation.
* **zoom.max_augment** (float, default: 30.0): Maximum zoom augmentation.
* **camera_not_moving_timeout** (float, default: 3.0): Time (seconds) to consider the camera stopped. If exceeded, control mode switches to idle.
* **last_position_command_timeout** (float, default: 10.0): Timeout (seconds) after the last position goal before stopping motion.
* **last_velocity_command_timeout** (float, default: 0.50): Timeout (seconds) after the last velocity command before stopping motion.
* **reject_new_goal** (bool, default: False): If true, new position goals are rejected while another goal is active.

## 1.2. Subscribed topics

* **~/cmd_vel** ([geometry_msgs/msg/Twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)): If velocity control is available on the AXIS camera, this topic is used to send commands for pan, tilt and zoom. The fields are interpreted as follows:
    * linear.x → controls pan (radians)
    * linear.y → controls tilt (radians)
    * angular.z → controls zoom (zoom increments; min and max are defined by the **min_augment** and **max_augment** parameters)

## 1.3. Published topics

* **~/joint_states** ([sensor_msgs/msg/JointState](https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html)): Publishes the position of the pan, tilt and zoom joints defined in the parameters.

* **~/status** (robotnik_sensors_msgs/msg/Axis): Publishes processed values received from the camera API, including: pan, tilt, zoom, focus, brightness, iris, autofocus and autoiris.

* **~/status_raw** (robotnik_sensors_msgs/msg/Axis): Publishes raw values directly received from the camera API, including: pan, tilt, zoom, focus, brightness, iris, autofocus and autoiris.

## 1.4. Services

* **~/stop_velocity_control** ([std_srvs/msg/Trigger](https://docs.ros2.org/foxy/api/std_srvs/srv/Trigger.html)): If velocity control is available on the AXIS camera, this service can be used to stop the velocity control.

## 1.5. Called services

None

## 1.6. Action servers

* **~/set_ptz** (robotnik_actuators_msgs/action/SetPtz): Action used to control the PTZ camera position.

* **~/home_ptz** (robotnik_actuators_msgs/action/SetPtz): Action used to move all joints to their zero (“home”) position. The goal message is ignored.

## 1.7. Action clients

None

## 1.8. Required tf Transforms

None

## 1.9. Provided tf Transforms

None

## 1.10. Bringup

``` bash
ros2 run axis_camera axis_ptz_node
```

or 

``` bash
ros2 launch axis_camera axis_ptz.launch.py
```

The launch file loads the config file in *config/<camera_model>.yaml*.
- The camera model is specified in the launch file.
- A separate configuration file should be created for each AXIS camera model.

---
# 2. axis_stream_node

The `axis_stream_node` runs the AXIS camera image stream.

## 1.1. Parameters

* **hostname** (string, default: "192.168.0.185"): IP address of the connected AXIS camera.
* **camera_number** (int, default: 1): Camera number associated to the camera.
* **desired_freq** (float, default: 30.0): Desired frequency (Hz) for the node loop.
* **camera_id** (string, default: "camera"): Camera ID.
* **camera_info_url** (string, default: 'package://axis_camera/data/default_calibration.yaml'): URL for the camera calibration data.
* **fps** (int, default: 0): Frames per second for the camera stream.
* **compression** (int, default: 0): Compression level for the video stream.
* **axis_frame_id** (string, default: 'axis_camera'): Frame ID for the AXIS camera.
* **profile** (string, default: 'Test'): Profile name for the camera settings.
* **timeout** (float, default: 5.0): Timeout duration (seconds) for camera connections and operations.
* **videocodec** (string, default: 'jpeg'): Video codec used for streaming.
* **resolution** (string, default: '1920x1080'): Resolution of the video stream.
* **initialization_delay** (float, default: 0.0): Delay (seconds) before initializing the camera.
* **reconnection_time** (float, default: 5.0): Time (seconds) to wait before attempting to reconnect to the camera.

## 1.2. Subscribed topics

None

## 1.3. Published topics

* **~/image_raw** ([sensor_msgs/msg/Image](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html)): Publishes the raw, uncompressed image stream from the camera.

* **~/image_raw/compressed** ([sensor_msgs/msg/CompressedImage](https://docs.ros2.org/foxy/api/sensor_msgs/msg/CompressedImage.html)): Publishes the same image stream in a compressed format (e.g., JPEG or PNG) to reduce bandwidth usage.

* **~/camera_info** ([sensor_msgs/msg/CameraInfo](https://docs.ros2.org/foxy/api/sensor_msgs/msg/CameraInfo.html)): Publishes the intrinsic parameters of the camera (e.g., focal length, distortion coefficients), which are required for tasks like image rectification or 3D reconstruction. These parameters are loaded from the file specified by the **camera_info_url** parameter.

**Note**: All these topics start publishing only when there is at least one subscriber connected.

## 1.4. Services

None

## 1.5. Called services

None

## 1.6. Action servers

None

## 1.7. Action clients

None

## 1.8. Required tf Transforms

None

## 1.9. Provided tf Transforms

None

## 1.10. Bringup

``` bash
ros2 run axis_camera axis_stream_node
```

or 

``` bash
ros2 launch axis_camera axis_stream.launch.py
```
