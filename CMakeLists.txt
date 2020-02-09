cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(peopleClassifier)

find_package(PCL 1.7 REQUIRED)
find_package(Torch REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (peopleClassifier MACOSX_BUNDLE peopleClassifier.cpp)
target_link_libraries (peopleClassifier ${PCL_LIBRARIES})
target_link_libraries(peopleClassifier "${TORCH_LIBRARIES}")
set_property(TARGET peopleClassifier PROPERTY CXX_STANDARD 14)
