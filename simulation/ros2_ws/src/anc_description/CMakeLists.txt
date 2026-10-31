cmake_minimum_required(VERSION 3.8)
project(anc_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(FILES "${PROJECT_SOURCE_DIR}/../../../../config/robot_parameters.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
ament_package()
