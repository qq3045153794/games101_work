cmake_minimum_required(VERSION 3.10)
project(ch4)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(ch4 main.cpp)

target_link_libraries(ch4 ${OpenCV_LIBRARIES})
