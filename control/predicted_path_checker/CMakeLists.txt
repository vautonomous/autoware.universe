cmake_minimum_required(VERSION 3.14)
project(predicted_path_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


ament_auto_add_library(predicted_path_checker SHARED
  src/predicted_path_checker_node/predicted_path_checker_node.cpp
  src/predicted_path_checker_node/collision_checker.cpp
  src/predicted_path_checker_node/utils.cpp

)

target_include_directories(predicted_path_checker
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(predicted_path_checker
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

rclcpp_components_register_node(predicted_path_checker
  PLUGIN "predicted_path_checker::PredictedPathCheckerNode"
  EXECUTABLE predicted_path_checker_node
)

if(BUILD_TESTING)

endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
