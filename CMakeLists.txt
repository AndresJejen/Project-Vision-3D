cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(main_ground_based_people_detection)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable (main_ground_based_people_detection main_ground_based_people_detection.cpp)
target_link_libraries (main_ground_based_people_detection ${PCL_LIBRARIES})