cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cylinder_segmentation)

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11

find_package(PCL 1.7 REQUIRED)

project( DisplayImage )
find_package( OpenCV REQUIRED )
include(FindPkgConfig)
PKG_CHECK_MODULES(OPENNI libopenni2)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/openni2
)

include_directories( ${OpenCV_INCLUDE_DIRS} )

LINK_DIRECTORIES(${OPENNI_LIBRARY_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (hsv_seg2 hsv_seg2.cpp)
target_link_libraries (hsv_seg2 ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OPENNI_LIBRARIES} )

