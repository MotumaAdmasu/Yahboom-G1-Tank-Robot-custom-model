# Install script for directory: /home/moti/ros2_ws/src/yahboom_g1_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/moti/ros2_ws/install/yahboom_g1_description")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description" TYPE DIRECTORY FILES "/home/moti/ros2_ws/src/yahboom_g1_description/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yahboom_g1_description" TYPE PROGRAM FILES "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/simple_tank_movement.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yahboom_g1_description" TYPE PROGRAM FILES
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/tank_controller_working.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/test_movement.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/mobile_tank_controller.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/real_movement_controller.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/correct_wheel_controller.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/proper_wheel_rolling.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description" TYPE DIRECTORY FILES "/home/moti/ros2_ws/src/yahboom_g1_description/urdf")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yahboom_g1_description" TYPE PROGRAM FILES
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/tank_controller_working.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/test_movement.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yahboom_g1_description" TYPE PROGRAM FILES
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/tank_controller_working.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/test_movement.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/mobile_tank_controller.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/real_movement_controller.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/correct_wheel_controller.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yahboom_g1_description" TYPE PROGRAM FILES
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/tank_controller_working.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/test_movement.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/mobile_tank_controller.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/real_movement_controller.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description" TYPE DIRECTORY FILES "/home/moti/ros2_ws/src/yahboom_g1_description/rviz")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yahboom_g1_description" TYPE PROGRAM FILES
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/tank_controller_working.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/test_movement.py"
    "/home/moti/ros2_ws/src/yahboom_g1_description/scripts/mobile_tank_controller.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description" TYPE DIRECTORY FILES "/home/moti/ros2_ws/src/yahboom_g1_description/config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/yahboom_g1_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/yahboom_g1_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description/environment" TYPE FILE FILES "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description/environment" TYPE FILE FILES "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description" TYPE FILE FILES "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description" TYPE FILE FILES "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description" TYPE FILE FILES "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description" TYPE FILE FILES "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description" TYPE FILE FILES "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_index/share/ament_index/resource_index/packages/yahboom_g1_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description/cmake" TYPE FILE FILES
    "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_core/yahboom_g1_descriptionConfig.cmake"
    "/home/moti/ros2_ws/build/yahboom_g1_description/ament_cmake_core/yahboom_g1_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yahboom_g1_description" TYPE FILE FILES "/home/moti/ros2_ws/src/yahboom_g1_description/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/moti/ros2_ws/build/yahboom_g1_description/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
