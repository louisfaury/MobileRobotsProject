# Install script for directory: /home/louis/Documents/Robotics/MobileRobotsProject/mbsysCopy/mbs_common

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/louis/Documents/Robotics/MobileRobotsProject/workR/Debug")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/louis/Documents/Robotics/MobileRobotsProject/workR/mbs_common/cmake_aux/listing/cmake_install.cmake")
  include("/home/louis/Documents/Robotics/MobileRobotsProject/workR/mbs_common/cmake_aux/flags/cmake_install.cmake")
  include("/home/louis/Documents/Robotics/MobileRobotsProject/workR/mbs_common/cmake_aux/libraries/cmake_install.cmake")
  include("/home/louis/Documents/Robotics/MobileRobotsProject/workR/mbs_common/mbs_numerics/cmake_install.cmake")
  include("/home/louis/Documents/Robotics/MobileRobotsProject/workR/mbs_common/mbs_struct/cmake_install.cmake")
  include("/home/louis/Documents/Robotics/MobileRobotsProject/workR/mbs_common/mbs_utilities/cmake_install.cmake")
  include("/home/louis/Documents/Robotics/MobileRobotsProject/workR/mbs_common/mbs_load_xml/cmake_install.cmake")
  include("/home/louis/Documents/Robotics/MobileRobotsProject/workR/mbs_common/mbs_realtime/cmake_install.cmake")
  include("/home/louis/Documents/Robotics/MobileRobotsProject/workR/mbs_common/mbs_module/cmake_install.cmake")

endif()
