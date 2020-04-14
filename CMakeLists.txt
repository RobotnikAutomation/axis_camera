cmake_minimum_required(VERSION 2.8.3)
project(axis_camera)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    std_msgs
    sensor_msgs camera_info_manager_py
    geometry_msgs
    rospy
    tf
    robotnik_msgs
    diagnostic_msgs
    diagnostic_updater
    std_srvs
    compressed_depth_image_transport
    compressed_image_transport
    theora_image_transport)
 
catkin_python_setup()

add_message_files(DIRECTORY msg 
FILES
   Axis.msg
   ptz.msg
   inputs_outputs.msg
)

add_service_files(DIRECTORY srv
FILES
   set_ptz.srv
   set_digital_output.srv
)

generate_messages(DEPENDENCIES sensor_msgs geometry_msgs std_msgs robotnik_msgs axis_camera)

catkin_package(
  CATKIN_DEPENDS 
    robotnik_msgs
    message_runtime 
)



catkin_install_python(PROGRAMS src/axis_camera/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/axis_camera/axis_node.py src/axis_camera/axis_node_gazebo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY launch config data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


