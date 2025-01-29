cmake_minimum_required(VERSION 3.4)
project(simple_admittance_controller)

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
)


find_package(Eigen3 REQUIRED)

find_package(Franka 0.7.0 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/simple_admittance_controller.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simple_admittance_controller
  CATKIN_DEPENDS
    controller_interface
    franka_hw
    hardware_interface
    pluginlib
    panda_ecat_comm
    data_extraction
    general_functionalities
    dynamic_reconfigure
  DEPENDS Franka
)

add_library(simple_admittance_controller
  src/simple_admittance_controller.cpp
)

add_dependencies(simple_admittance_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(simple_admittance_controller PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(simple_admittance_controller SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(simple_admittance_controller PUBLIC
  include
)

## Installation
install(TARGETS simple_admittance_controller
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
install(FILES simple_admittance_controller_plugin.xml
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
  add_format_target(simple_admittance_controller FILES ${SOURCES} ${HEADERS})
  add_tidy_target(simple_admittance_controller
    FILES ${SOURCES}
    DEPENDS simple_admittance_controller
  )
endif()
