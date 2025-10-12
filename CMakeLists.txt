cmake_minimum_required(VERSION 3.5)
project(krabi_description)

find_package(ament_cmake REQUIRED)

#install(
#  DIRECTORY models
#  DESTINATION share/${PROJECT_NAME}
#)
#install(DIRECTORY
#  worlds
#  DESTINATION share/${PROJECT_NAME}/
#)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf params
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY models params
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds params
  DESTINATION share/${PROJECT_NAME})

#ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/env_variables.bash")
ament_package()
