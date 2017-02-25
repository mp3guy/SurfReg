cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(SurfReg)

find_package(PCL 1.7 REQUIRED)
find_package(Boost COMPONENTS serialization filesystem system REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3")

add_executable (SurfReg SurfReg.cpp)
target_link_libraries (SurfReg  ${Boost_LIBRARIES} ${PCL_LIBRARIES})
