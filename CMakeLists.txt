cmake_minimum_required(VERSION 2.8.3)
project(ed_control)
add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_sdk
  rospy
  std_msgs
  geometry_msgs
  octomap_msgs
  tf
  moveit_core
  moveit_ros_planning_interface
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
  cv_bridge
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
)
find_package(octomap REQUIRED)

find_package(OpenCV 3 REQUIRED)
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs

 )

catkin_package(
#  LIBRARIES ed_control
 CATKIN_DEPENDS roscpp
  dynamixel_sdk
  actionlib
  trajectory_msgs
  hector_uav_msgs
  control_msgs
  sensor_msgs
  bebop_msgs
#  DEPENDS system_lib
   DEPENDS octomap
 moveit_core
 moveit_ros_planning_interface
dynamixel_workbench_msgs 
)
include_directories(
   include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)
find_package(OpenCV 2)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(acontrol_arm src/acontrol_arm.cpp)
        
target_link_libraries(acontrol_arm ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_dependencies(acontrol_arm ed_control_generate_messages_cpp  ${OpenCV_LIBRARIES})


add_executable(uav_control src/uav_control.cpp)
        
target_link_libraries(uav_control ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_dependencies(uav_control ed_control_generate_messages_cpp  ${OpenCV_LIBRARIES})


add_executable(summit_control src/summit_control.cpp)
                              
target_link_libraries(summit_control ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(summit_control ed_control_generate_messages_cpp  ${OpenCV_LIBRARIES})


add_executable(pioneer_p3dx_control src/pioneer_p3dx_control.cpp)
                              
target_link_libraries(pioneer_p3dx_control ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(pioneer_p3dx_control ed_control_generate_messages_cpp  ${OpenCV_LIBRARIES})



add_executable(pioneer_p3dx_control_simm src/pioneer_p3dx_control_simm.cpp)
                              
target_link_libraries(pioneer_p3dx_control_simm ${catkin_LIBRARIES} )

add_dependencies(pioneer_p3dx_control_simm ed_control_generate_messages_cpp  ${OpenCV_LIBRARIES})



add_executable(mx_motors_control src/mx_arm_control.cpp)
   
target_link_libraries(mx_motors_control ${catkin_LIBRARIES})

add_dependencies(mx_motors_control ed_control_generate_messages_cpp )


add_executable(pro_motors_control src/pro_arm_control.cpp)
   
target_link_libraries(pro_motors_control ${catkin_LIBRARIES})

add_dependencies(pro_motors_control ed_control_generate_messages_cpp )


add_executable(single_motor_joy_control src/single_motor_joy_control.cpp)
   
target_link_libraries(single_motor_joy_control ${catkin_LIBRARIES})

add_dependencies(single_motor_joy_control ed_control_generate_messages_cpp )



add_executable(dokku_motors_control src/dokku_arm_control.cpp)
   
target_link_libraries(dokku_motors_control ${catkin_LIBRARIES})

add_dependencies(dokku_motors_control ed_control_generate_messages_cpp )



add_executable(pro_control_sdk src/pro_control_sdk.cpp)
   
target_link_libraries(pro_control_sdk ${catkin_LIBRARIES})

add_dependencies(pro_control_sdk ed_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


add_executable(pro_control_sdk_sp src/pro_control_sdk_sp.cpp)
   
target_link_libraries(pro_control_sdk_sp ${catkin_LIBRARIES})

add_dependencies(pro_control_sdk_sp ed_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


add_executable(opencv_image_filter src/image_filter.cpp)
   
target_link_libraries(opencv_image_filter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(opencv_image_filter ed_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})


add_executable(arduino_control src/arduino_control.cpp)
   
target_link_libraries(arduino_control ${catkin_LIBRARIES})

add_dependencies(arduino_control ed_control_generate_messages_cpp )


