cmake_minimum_required(VERSION 3.4)
project(data_extraction)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  panda_ecat_comm
  franka_hw
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    panda_ecat_comm
    franka_hw
  DEPENDS Franka
)

#add_library(data_extraction
  #src/data_extraction.cpp
#)

#add_dependencies(data_extraction
  #${${PROJECT_NAME}_EXPORTED_TARGETS}
  #${catkin_EXPORTED_TARGETS}
#)

#target_link_libraries(data_extraction PUBLIC
  #${Franka_LIBRARIES}
  #${catkin_LIBRARIES}
#)

#target_include_directories(data_extraction SYSTEM PUBLIC
  #${Franka_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIRS}
  #${catkin_INCLUDE_DIRS}
#)
#target_include_directories(data_extraction PUBLIC
  #include
#)

## Installation
#install(TARGETS data_extraction
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#install(DIRECTORY launch
  #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#install(DIRECTORY config
  #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#install(FILES data_extraction_plugin.xml
  #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

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
  add_format_target(data_extraction FILES ${SOURCES} ${HEADERS})
  add_tidy_target(data_extraction
    FILES ${SOURCES}
    DEPENDS data_extraction
  )
endif()
