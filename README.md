# axis_camera

## Overview

This ROS_ package provides an [Axis network camera](https://www.axis.com/products/network-cameras) driver, written in Python.

ROS wiki documentation: [axis_camera](http://wiki.ros.org/axis_camera).

This driver is under active development. Its ROS interfaces are relatively stable, but may still change.  

## Nodes

- axis_node.py: Version 2 of the axis.py node. It works based on profiles + params. It is necessary to have the profiles defined. **Sets up axis camera and ptz server.**

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

### Parameters
#### Published Topics
* ~zoom_parameters (robotnik_msgs/CameraParameters)
  Zoom parameters publisher, containing min_zoom_step, max_zoom_augment, min_zoom_augment and a list of available augments
* ~camera_params (robotnik_msgs/Axis)


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

### Controlling your camera

You can send your camera PTZ commands via the `/axis/axis_camera/ptz_command` topic:

```
rostopic pub /axis/axis_camera/ptz_command robotnik_msgs/ptz "pan: 0.5
tilt: -1.0
zoom: 2.0
relative: false" 
```

* params pan and tilt as float (radians)
* param zoom as float (proportional zoom between min_zoom_augment and max_zoom_augment)
* param relative as bool (increases the current pan,tilt,zoom relative to the current values)
