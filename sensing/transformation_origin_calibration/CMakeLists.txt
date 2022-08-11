cmake_minimum_required(VERSION 3.14)
project(transformation_origin_calibration)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

set(PACKAGE_HEADERS
  include/transformation_origin_calibration/transformation_origin_calibration_core.hpp
)

ament_auto_add_library(transformation_origin_calibration_node SHARED
        src/transformation_origin_calibration_core.cpp
  ${PACKAGE_HEADERS}
)

target_link_libraries(transformation_origin_calibration_node
  Geographic
)

rclcpp_components_register_node(transformation_origin_calibration_node
  PLUGIN "transformation_origin_calibration::TransformationOriginCalibration"
  EXECUTABLE transformation_origin_calibration
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
