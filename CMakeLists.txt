cmake_minimum_required (VERSION 2.6)
project(locomotion-planner)
# The version number.
set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)
set ( libipopt.a )


find_package(catkin REQUIRED
  roscpp
  locomotion-viewer
  )

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp locomotion-viewer
)

# configure a header file to pass some of the CMake settings
# to the source code
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_BINARY_DIR}" /usr/local/include/coin/)						

# creation of an executable
add_executable(NL_planner motion_planner_example.cpp src/locomotion-planner.cpp)
target_link_libraries(NL_planner ${catkin_LIBRARIES} ipopt)
set_target_properties(NL_planner PROPERTIES COMPILE_FLAGS "-std=c++11 -o3" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin") 
