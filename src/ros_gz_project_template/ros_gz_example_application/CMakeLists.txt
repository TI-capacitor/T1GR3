cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)

ament_package()
# Install Python script
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/autonomous_drone.py
  DESTINATION lib/${PROJECT_NAME}
)