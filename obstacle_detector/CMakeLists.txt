cmake_minimum_required(VERSION 3.5)
project(obstacle_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/obstacle_detector/ObstacleDetectorNode.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
rclcpp_components_register_nodes(${PROJECT_NAME} "obstacle_detector::AvoidanceNode")

add_executable(obstacle_detector_main src/obstacle_detector_main.cpp)
ament_target_dependencies(obstacle_detector_main ${dependencies})
target_link_libraries(obstacle_detector_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  obstacle_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_dependencies(${dependencies})
ament_package()
