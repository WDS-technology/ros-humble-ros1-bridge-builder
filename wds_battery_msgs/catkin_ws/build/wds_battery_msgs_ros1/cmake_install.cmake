# Install script for directory: /home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/src/wds_battery_msgs_ros1

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wds_battery_msgs/msg" TYPE FILE FILES "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/src/wds_battery_msgs_ros1/msg/WdsBattery.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wds_battery_msgs/cmake" TYPE FILE FILES "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/build/wds_battery_msgs_ros1/catkin_generated/installspace/wds_battery_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/devel/include/wds_battery_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/devel/share/roseus/ros/wds_battery_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/devel/share/common-lisp/ros/wds_battery_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/devel/share/gennodejs/ros/wds_battery_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/devel/lib/python2.7/dist-packages/wds_battery_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/devel/lib/python2.7/dist-packages/wds_battery_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/build/wds_battery_msgs_ros1/catkin_generated/installspace/wds_battery_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wds_battery_msgs/cmake" TYPE FILE FILES "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/build/wds_battery_msgs_ros1/catkin_generated/installspace/wds_battery_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wds_battery_msgs/cmake" TYPE FILE FILES
    "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/build/wds_battery_msgs_ros1/catkin_generated/installspace/wds_battery_msgsConfig.cmake"
    "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/build/wds_battery_msgs_ros1/catkin_generated/installspace/wds_battery_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wds_battery_msgs" TYPE FILE FILES "/home/root/wds/ros-humble-ros1-bridge-builder/wds_battery_msgs/catkin_ws/src/wds_battery_msgs_ros1/package.xml")
endif()

