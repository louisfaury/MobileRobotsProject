# Install script for directory: /home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR/build/Debug")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR/build/cmake_aux/flags/cmake_install.cmake")
  INCLUDE("/home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR/build/cmake_aux/listing/cmake_install.cmake")
  INCLUDE("/home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR/build/cmake_aux/libraries/cmake_install.cmake")
  INCLUDE("/home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR/build/cmake_aux/make_opt/cmake_install.cmake")
  INCLUDE("/home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR/build/mbs_common/cmake_install.cmake")
  INCLUDE("/home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR/build/symbolicR/cmake_install.cmake")
  INCLUDE("/home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR/build/userfctR/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/gregoire/Documents/MobileRobots/MobileRobotsProject/workR/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
