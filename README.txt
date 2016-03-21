Nodes
-----

- axis_io.py: Node to access the I/O of the Axis. 

- axis_io_test.py: Example program to access the I/O of the camera

- axis_ptz.py: Node to set and read the pan/tilt/zoom the camera

- axis_sound.py: Test node to access the sound functionalities of the camera

- axis_video_manager.py:

- axis.py: Node to read and publish the video stream of the camera

- axis_v2.py: Version 2 of the axis.py node. It works based on profiles + params. It is necessary to have the profiles defined. 



Configuration
---------------

If we want to use camera_info publication it is important that configuration .yaml has the same resolution than the resolution set in the camera (using or not profiles)




