cmake_minimum_required(VERSION 3.5)

project(ros_gz_crazyflie_gazebo)

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY models/worlds/
  DESTINATION share/${PROJECT_NAME}/models/worlds
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
