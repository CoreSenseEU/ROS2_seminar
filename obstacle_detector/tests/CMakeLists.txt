
ament_add_gtest(obstacle_detector_test obstacle_detector_test.cpp)
ament_target_dependencies(obstacle_detector_test ${dependencies})
target_link_libraries(obstacle_detector_test ${PROJECT_NAME})
