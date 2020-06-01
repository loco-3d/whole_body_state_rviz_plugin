CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
project(state_rviz_plugin)

# Set up project properties
SET(PROJECT_NAME state_rviz_plugin)
SET(PROJECT_DESCRIPTION "State rviz plugin")

# Print initial message
MESSAGE(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE("Copyright (C) 2020-2022 University of Edinburgh")
MESSAGE("All rights reserved.")
MESSAGE("Released under the BSD 3-Clause License.")

# Set a default build type to 'Release' if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  state_msgs
  roscpp
  rviz)

FIND_PACKAGE(Boost REQUIRED COMPONENTS system)

FIND_PACKAGE(pinocchio REQUIRED)

IF(rviz_QT_VERSION VERSION_LESS "5")
  MESSAGE(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  FIND_PACKAGE(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  INCLUDE(${QT_USE_FILE})

  qt4_wrap_cpp(MOC_FILES
    include/state_rviz_plugin/PointVisual.h
    include/state_rviz_plugin/LineVisual.h
    include/state_rviz_plugin/ArrowVisual.h
    include/state_rviz_plugin/PolygonVisual.h
    include/state_rviz_plugin/WholeBodyStateDisplay.h
    # include/state_rviz_plugin/WholeBodyTrajectoryDisplay.h
    # include/state_rviz_plugin/ReducedTrajectoryDisplay.h
    # include/state_rviz_plugin/FootstepRegionDisplay.h
    OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
ELSE()
  MESSAGE(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  FIND_PACKAGE(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  SET(QT_LIBRARIES Qt5::Widgets)

  qt5_wrap_cpp(MOC_FILES
    include/state_rviz_plugin/PointVisual.h
    include/state_rviz_plugin/LineVisual.h
    include/state_rviz_plugin/ArrowVisual.h
    include/state_rviz_plugin/PolygonVisual.h
    include/state_rviz_plugin/WholeBodyStateDisplay.h
    # include/state_rviz_plugin/WholeBodyTrajectoryDisplay.h
    # include/state_rviz_plugin/ReducedTrajectoryDisplay.h
    # include/state_rviz_plugin/FootstepRegionDisplay.h
    OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
ENDIF()

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pinocchio #state_msgs #${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz)


MESSAGE("------------${catkin_INCLUDE_DIRS}")
MESSAGE("------------${state_msgs_EXPORTED_TARGETS}")
MESSAGE("------------${pinocchio_LIBRARIES}")
INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

LINK_DIRECTORIES(${catkin_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS})
                 

SET(SOURCE_FILES
  # src/DisplayInterface.cpp
  src/PointVisual.cpp
  src/LineVisual.cpp
  src/ArrowVisual.cpp
  src/PolygonVisual.cpp
  src/WholeBodyStateDisplay.cpp
  # src/WholeBodyTrajectoryDisplay.cpp
  # src/ReducedTrajectoryDisplay.cpp
  # src/TerrainMapDisplay.cpp
  # src/ObstacleMapDisplay.cpp
  # src/FootstepRegionDisplay.cpp
  ${MOC_FILES})


ADD_LIBRARY(${PROJECT_NAME}  ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ${QT_LIBRARIES}
                                       ${Boost_LIBRARIES}
                                       ${catkin_LIBRARIES}
                                       ${pinocchio_LIBRARIES})
# ADD_DEPENDENCIES(${PROJECT_NAME}  ${state_msgs_EXPORTED_TARGETS})

INSTALL(FILES ${CMAKE_SOURCE_DIR}/plugin_description.xml DESTINATION share/${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
            DESTINATION DESTINATION include
            FILES_MATCHING PATTERN "*.h*")