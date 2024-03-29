CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
project(whole_body_state_rviz_plugin)

# Set up project properties
SET(PROJECT_NAME whole_body_state_rviz_plugin)
SET(PROJECT_DESCRIPTION "Whole-body state rviz plugin")

# Print initial message (STATUS to avoid triggering warnings)
MESSAGE(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE(STATUS "Copyright (C) 2020-2023 Heriot-Watt Unversity, University of Edinburgh")
MESSAGE(STATUS "                        Istituto Italiano di Tecnologia, University of Oxford")
MESSAGE(STATUS "All rights reserved.")
MESSAGE(STATUS "Released under the BSD 3-Clause License.")

# Set a default build type to 'Release' if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# C++14 (Kinetic+)
IF(CMAKE_CXX_STANDARD GREATER 14)
  MESSAGE(WARNING "OGRE cannot be built with C++17 or higher. Force setting C++14.")
  SET(CMAKE_CXX_STANDARD 14)
ENDIF()

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  whole_body_state_msgs
  roscpp
  rviz)

# Ensure version of whole_body_state_msgs is at least 1.0.0
IF(NOT ${whole_body_state_msgs_VERSION} VERSION_GREATER "0.9.99")
  MESSAGE(FATAL_ERROR "whole_body_state_msgs version 1.0.0 or greater required.")
ENDIF()

FIND_PACKAGE(Boost REQUIRED COMPONENTS system)

FIND_PACKAGE(pinocchio REQUIRED)

IF(rviz_QT_VERSION VERSION_LESS "5")
  MESSAGE(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  FIND_PACKAGE(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  INCLUDE(${QT_USE_FILE})

  qt4_wrap_cpp(MOC_FILES
    include/whole_body_state_rviz_plugin/PointVisual.h
    include/whole_body_state_rviz_plugin/LineVisual.h
    include/whole_body_state_rviz_plugin/ArrowVisual.h
    include/whole_body_state_rviz_plugin/PolygonVisual.h
    include/whole_body_state_rviz_plugin/ConeVisual.h
    include/whole_body_state_rviz_plugin/PinocchioLinkUpdater.h
    include/whole_body_state_rviz_plugin/WholeBodyStateDisplay.h
    include/whole_body_state_rviz_plugin/WholeBodyTrajectoryDisplay.h
    OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
ELSE()
  MESSAGE(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  FIND_PACKAGE(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  SET(QT_LIBRARIES Qt5::Widgets)

  qt5_wrap_cpp(MOC_FILES
    include/whole_body_state_rviz_plugin/PointVisual.h
    include/whole_body_state_rviz_plugin/LineVisual.h
    include/whole_body_state_rviz_plugin/ArrowVisual.h
    include/whole_body_state_rviz_plugin/PolygonVisual.h
        include/whole_body_state_rviz_plugin/ConeVisual.h
    include/whole_body_state_rviz_plugin/PinocchioLinkUpdater.h
    include/whole_body_state_rviz_plugin/WholeBodyStateDisplay.h
    include/whole_body_state_rviz_plugin/WholeBodyTrajectoryDisplay.h
    OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
ENDIF()

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS)


INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

SET(SOURCE_FILES
  src/PointVisual.cpp
  src/LineVisual.cpp
  src/ArrowVisual.cpp
  src/PolygonVisual.cpp
  src/ConeVisual.cpp
  src/PinocchioLinkUpdater.cpp
  src/WholeBodyStateDisplay.cpp
  src/WholeBodyTrajectoryDisplay.cpp
  ${MOC_FILES})


ADD_LIBRARY(${PROJECT_NAME}  ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ${QT_LIBRARIES}
                                       ${Boost_LIBRARIES}
                                       ${catkin_LIBRARIES}
                                       pinocchio::pinocchio)
#TARGET_COMPILE_OPTIONS(${PROJECT_NAME} PRIVATE -Wno-ignored-attributes)  # Silence Eigen::Tensor warnings

INSTALL(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
