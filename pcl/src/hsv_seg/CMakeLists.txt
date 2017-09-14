cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cylinder_segmentation)

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (hsv_seg hsv_seg.cpp)
target_link_libraries (hsv_seg ${PCL_LIBRARIES})

