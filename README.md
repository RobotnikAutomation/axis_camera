# axis_camera

## Overview

This ROS_ package provides an [Axis network camera](https://www.axis.com/products/network-cameras) driver, written in Python.

ROS wiki documentation: [axis_camera](http://wiki.ros.org/axis_camera).

This driver is under active development. Its ROS interfaces are relatively stable, but may still change.  

## Nodes

- `axis_node.py`: Version 2 of the axis.py node. Works based on profiles + params. It is necessary to have the profiles defined. Sets up axis camera and PTZ server.
- `axis_ptz_node.py`: PTZ control node. Controls pan, tilt, zoom, focus and iris. See [PTZ node](#ptz-node) for full documentation.
- `axis_stream_node.py`: Image streaming node.

## Configuration

If we want to use camera_info publication it is important that configuration .yaml has the same resolution than the resolution set in the camera (using or not profiles)

### How to set up your axis camera

> Note: if you are using AXIS M5054 you can use p5534 configuration files.

#### 1. Change your model name on launch file

In `launch/axis.launch`:

```yaml
<arg name="camera_model" default="axis_model"/>
```

```yaml
<param name="camera_model" value="axis_model"/>
```

#### 2. Create parameter files for your camera model

- `data/axis_model.ayml`: camera parameters
- `config/axis_model_ptz_config.yaml`: ptz control parameters.
  Define absolute zoom range and standardized zoom range. For example to control your camera with zoom augments in the range [x0 , x30]:

  ```
  min_zoom_step: 1
  min_zoom_augment: 0.0
  max_zoom_augment: 30.0
  ```

### Testing your camera

There are three different launch files in this package.

> Note: if you encounter this error while trying to run any of the following commands just type `export ROS_NAMESPACE=axis` in your console.

- For running the camera control node and starting the live view GUI:
```
roslaunch axis_camera start_axis_and_video.launch
```

- For running only the camera control node:
```
roslaunch axis_camera axis.launch
```

- For running only the live view GUI (camera control node must be running):
```
roslaunch axis_camera image_view.launch
```

- You can also run image_view directly (setting the image topic to `/axis/image_color`):
```
rosrun image_view image_view image:=/axis/image_color
```

---

## PTZ node

`axis_ptz_node.py` controls pan, tilt, zoom, focus and iris of an Axis PTZ camera.

Launch with:
```
roslaunch axis_camera axis_ptz.launch ip_address:=<camera_ip>
```

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `hostname` | string | `192.168.1.205` | Camera IP address or hostname |
| `username` | string | `root` | Camera username (used when `enable_auth` is true) |
| `password` | string | `R0b0tn1K` | Camera password (used when `enable_auth` is true) |
| `enable_auth` | bool | `false` | Enable HTTP digest authentication |
| `camera_model` | string | `axis_m5525` | Camera model name (used for parameter file lookup) |
| `ptz_rate` | float | `5.0` | Control loop rate (Hz) |
| `send_constantly` | bool | `false` | Send PTZ commands continuously even when there is no new command |
| `pan_joint` | string | `pan` | Name of the pan joint in joint state messages |
| `tilt_joint` | string | `tilt` | Name of the tilt joint in joint state messages |
| `zoom_joint` | string | `zoom` | Name of the zoom joint in joint state messages |
| `min_pan_value` | float | `-2.97` | Minimum pan limit (radians) |
| `max_pan_value` | float | `2.97` | Maximum pan limit (radians) |
| `min_tilt_value` | float | `0` | Minimum tilt limit (radians) |
| `max_tilt_value` | float | `1.57` | Maximum tilt limit (radians) |
| `min_zoom_value` | float | `0` | Minimum zoom value in camera units |
| `max_zoom_value` | float | `20000` | Maximum zoom value in camera units |
| `min_zoom_augment` | float | `0.0` | Minimum zoom augment (user-facing) |
| `max_zoom_augment` | float | `30.0` | Maximum zoom augment (user-facing) |
| `min_zoom_step` | float | `1` | Zoom step size |
| `invert_pan` | bool | `false` | Invert pan axis |
| `invert_tilt` | bool | `false` | Invert tilt axis |
| `pan_offset` | float | `0.0` | Pan offset in radians (for non-centred mounting) |
| `tilt_offset` | float | `0.0` | Tilt offset in radians (for non-centred mounting) |
| `use_control_timeout` | bool | `false` | Release PTZ control after `control_timeout_value` seconds of inactivity |
| `control_timeout_value` | float | `5.0` | Inactivity timeout in seconds (only used when `use_control_timeout` is true) |
| `iris_two_step_control` | bool | `false` | Send `autoiris=off` as a separate request before each manual iris command. Helps prevent automatic reversion on some camera models (e.g. Axis P5676-LE). |

### Published Parameters

* `~device/model` (string) — Camera model detected at startup.
* `~device/serial` (string) — Camera serial number detected at startup.
* `~device/firmware` (string) — Camera firmware version detected at startup.

### Published Topics

* `~camera_params` (`robotnik_msgs/Axis`) — Current PTZ state. `focus` and `iris` fields are published as percentage (0–100).
* `~camera_parameters` (`robotnik_msgs/CameraParameters`) — Zoom range and step configuration.
* `~joint_states` (`sensor_msgs/JointState`) — Pan, tilt and zoom as joint positions.

### Subscribed Topics

* `~ptz_command` (`robotnik_msgs/ptz`) — PTZ command. Pan and tilt in radians, zoom as augment value.

### Services

#### `~home_ptz` (`std_srvs/Empty`)
Moves the camera to the home position (pan=0, tilt=0, zoom=0).
```
rosservice call /axis_camera_ptz/home_ptz
```

#### `~get_device_info` (`robotnik_msgs/GetAxisDeviceInfo`)
Returns camera device information (model, serial number, firmware version).
```
rosservice call /get_device_info
```

#### `~set_focus` (`robotnik_msgs/SetCameraFocus`)
Controls camera focus.

Request fields:
* `value` (`float32`): target focus in **percentage** (`0.0` = nearest, `100.0` = furthest). Ignored when `auto` is true.
* `auto` (`bool`): if `true`, enable autofocus; if `false`, apply manual focus `value`.

Examples:
```bash
# Enable autofocus
rosservice call /axis_camera_ptz/set_focus "value: 0.0
auto: true"

# Set manual focus to 50%
rosservice call /axis_camera_ptz/set_focus "value: 50.0
auto: false"
```

#### `~set_iris` (`robotnik_msgs/SetCameraIris`)
Controls camera iris (aperture).

Request fields:
* `value` (`float32`): target iris opening in **percentage** (`0.0` = closed, `100.0` = fully open). Ignored when `auto` is true.
* `auto` (`bool`): if `true`, enable autoiris; if `false`, apply manual iris `value`.

Examples:
```bash
# Enable autoiris
rosservice call /axis_camera_ptz/set_iris "value: 0.0
auto: true"

# Set manual iris to 75%
rosservice call /axis_camera_ptz/set_iris "value: 75.0
auto: false"
```

> **Note on service names**: service names depend on the node namespace. Run `rosservice list` to get the exact names in your setup.

### Controlling pan, tilt and zoom

Send PTZ commands via the `~ptz_command` topic:

```
rostopic pub /axis/axis_camera/ptz_command robotnik_msgs/ptz "pan: 0.5
tilt: -1.0
zoom: 2.0
relative: false" 
```

* `pan` and `tilt` as float (radians)
* `zoom` as float (proportional value between `min_zoom_augment` and `max_zoom_augment`)
* `relative` as bool — if `true`, values are added to the current position

### Focus and iris behaviour

* Focus and iris limits are **read from the camera at startup** and used to map between raw camera units and percentage.
* Focus and iris values in `~camera_params` are always in **percentage (0–100)**.
* If the camera does not report focus or iris support, a warning is logged and the respective service returns an error without crashing the node.
* On some camera models (e.g. Axis P5676-LE) manual iris control may not hold reliably due to firmware behaviour. Enable `iris_two_step_control` to mitigate this.
