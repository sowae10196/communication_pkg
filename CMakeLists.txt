cmake_minimum_required(VERSION 3.0.2)
project(communication_pkg)

add_compile_options(-std=c++11)

find_package(kusv_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  kusv_msgs
  serial
)

set(communication_SRCS
  src/communication.cpp
  include/communication_pkg/communication.hpp
)

add_library(${PROJECT_NAME} ${communication_SRCS})

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES communication_pkg
 CATKIN_DEPENDS roscpp std_msgs kusv_msgs serial
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(communication_main src/communication_main.cpp)
add_dependencies(communication_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(communication_main
  ${catkin_LIBRARIES}
)

add_executable(testFunctions src/testFunctions.cpp)
add_dependencies(testFunctions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testFunctions
  ${catkin_LIBRARIES}
)

# install (FILES include/communication_pkg/communication.hpp
#   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/communication_pkg
# )