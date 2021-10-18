cmake_minimum_required(VERSION 2.8.3)
project(blvr_controller)

add_compile_options(--std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/blvr_controller.cpp
  src/blvr_interface.cpp
  src/blvr_comunicator.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

