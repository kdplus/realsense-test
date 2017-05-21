cmake_minimum_required(VERSION 2.8.11)
project(gui)


find_package(OpenCV REQUIRED)
find_library(Realsense_LIBS realsense)
include_directories(${OpenCV_INCLUDE_DIRS})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11 ")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} " )

file(GLOB main_files ./BGR_sample.cpp)
set(SOURCE_FILES ${main_files} )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(gui ${SOURCE_FILES})
target_link_libraries(gui ${OpenCV_LIBS} ${Realsense_LIBS} )
