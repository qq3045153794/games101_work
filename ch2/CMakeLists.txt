cmake_minimum_required(VERSION 3.10)
project(RasterizerCh2)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)

add_executable(RasterizerCh2 main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_link_libraries(RasterizerCh2 ${OpenCV_LIBRARIES})
