cmake_minimum_required(VERSION 3.4)
project(trial_controller_velocity)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  franka_hw
  hardware_interface
  pluginlib
  panda_ecat_comm
  data_extraction
  general_functionalities
  dynamic_reconfigure
  qpoases
)


find_package(Eigen3 REQUIRED)

find_package(Franka 0.7.0 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/trial_controller_velocity.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trial_controller_velocity
  CATKIN_DEPENDS
    controller_interface
    franka_hw
    hardware_interface
    pluginlib
    panda_ecat_comm
    data_extraction
    general_functionalities
    dynamic_reconfigure
    qpoases
  DEPENDS Franka
)

add_library(trial_controller_velocity
  src/trial_controller_velocity.cpp
)

add_dependencies(trial_controller_velocity
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(trial_controller_velocity PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/optimizer6/target/release/liboptimizer6.a
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(trial_controller_velocity SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(trial_controller_velocity PUBLIC
  include
  optimizer6
)

## Installation
install(TARGETS trial_controller_velocity
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES trial_controller_velocity_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(trial_controller_velocity FILES ${SOURCES} ${HEADERS})
  add_tidy_target(trial_controller_velocity
    FILES ${SOURCES}
    DEPENDS trial_controller_velocity
  )
endif()
