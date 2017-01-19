cmake_minimum_required(VERSION 2.8.12)
project(depth_clustering)

include(DistVersion.cmake)
system_info(DISTRO)
message(STATUS "DISTRO: ${DISTRO}")

if(NOT CMAKE_BUILD_TYPE)
  set(POSSIBLE_BUILD_TYPES "Debug Release RelWithDebInfo MinSizeRel")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: ${POSSIBLE_BUILD_TYPES}." FORCE)
endif(NOT CMAKE_BUILD_TYPE)
message(STATUS "CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(WARNING "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
    message(STATUS "Please use a different C++ compiler.")
endif()

# enable warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include(CTest)
IF(IS_DIRECTORY "/usr/src/gtest/")
  MESSAGE(STATUS "Found google test sources in /usr/src/gtest/")

  ADD_SUBDIRECTORY(/usr/src/gtest/ gtest)

  # mimick the behaviour of find_package(GTest)
  SET(GTEST_FOUND TRUE)
  SET(GTEST_BOTH_LIBRARIES gtest gtest_main)
  SET(GTEST_LIBRARIES gtest)
  SET(GTEST_MAIN_LIBRARIES gtest_main)
ELSE()
  find_package(GTest)
ENDIF()

set(Boost_USE_STATIC_LIBS OFF)

option(BUILD_WITH_COVERAGE "Enable compilation with coverage information." OFF)
if(BUILD_WITH_COVERAGE)
  message(STATUS "Enabled generation of code coverage information with gcc.")
  set(CMAKE_CXX_FLAGS "-fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS}")
endif()

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)
find_package(PCL QUIET COMPONENTS common io)
find_package(QGLViewer REQUIRED)

# find correct qt version
if(${DISTRO} MATCHES "14.04")
  find_package(Qt4 REQUIRED COMPONENTS QtCore QtXml QtOpenGL QtGui)
  include_directories(${QT_INCLUDES})
  set(MY_QT_LIBRARIES
      Qt4::QtCore
      Qt4::QtXml
      Qt4::QtOpenGL
      Qt4::QtGui)
elseif(${DISTRO} MATCHES "16.04")
  find_package(Qt5 REQUIRED COMPONENTS Core Xml OpenGL Gui Widgets)
  include_directories(${Qt5Core_INCLUDE_DIRS}
                      ${Qt5Xml_INCLUDE_DIRS}
                      ${Qt5Gui_INCLUDE_DIRS}
                      ${Qt5Widgets_INCLUDE_DIRS}
                      ${Qt5OpenGL_INCLUDE_DIRS})
  set(MY_QT_LIBRARIES
      ${Qt5Widgets_LIBRARIES}
      ${Qt5Core_LIBRARIES}
      ${Qt5Gui_LIBRARIES}
      ${Qt5Xml_LIBRARIES}
      ${Qt5OpenGL_LIBRARIES})
endif()

# some status messages
message(STATUS "Linking against Qt libs: ${MY_QT_LIBRARIES}")
message(STATUS "Linking against QGlViewer lib: ${QGLVIEWER_LIBRARY}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin QUIET COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
  tf
  eigen_conversions
)

# add some useful constants to configuration
configure_file(config/cmake_config.h.in
               ${PROJECT_SOURCE_DIR}/config/cmake_config.h)
include_directories(config)


if (${PCL_FOUND})
  message(STATUS "PCL is found. PCL-related code will be built.")
  if (${CMAKE_BUILD_TYPE} MATCHES "Debug" AND ${DISTRO} MATCHES "14.04")
    message(FATAL_ERROR
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
            "You are building PCL in Debug mode with C++11 enabled.\n"
            "This causes segmentation fault due to a bug in PCL.\n"
            "Build in Release mode to avoid this bug.\n"
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
  endif()
  add_definitions(-DPCL_FOUND)
else()
  message(WARNING "PCL NOT FOUND! PCL-related code WILL NOT be built")
endif()

if(${roscpp_FOUND})
  message(STATUS "ROS found, building ROS related parts")
  set(ROS_FOUND YES)
else()
  message(WARNING "ROS NOT FOUND. NOT building ROS related parts!")
  set(ROS_FOUND NO)
endif()

catkin_package(
 INCLUDE_DIRS src
 LIBRARIES
      cloud
      identifiable
      ground_remove
      difference
      image_labeler
      projections
      ros_bridge
      velodyne_utils
      folder_reader
      visualization
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs nav_msgs message_filters tf
)

include_directories(
  src
  SYSTEM ${EIGEN3_INCLUDE_DIR}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}
  SYSTEM ${QGLVIEWER_INCLUDE_DIR}
  SYSTEM ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
add_subdirectory(examples)

message(STATUS "BUILD_TESTING: ${BUILD_TESTING}")
if (${BUILD_TESTING})
  add_subdirectory(test)
endif()
