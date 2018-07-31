cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
set(DEPENDENCIES realsense2 ${PCL_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")



add_executable (pcd_write main.cpp)
target_link_libraries (pcd_write ${DEPENDENCIES})
