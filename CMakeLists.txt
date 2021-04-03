cmake_minimum_required(VERSION 3.10)
project(pixel_to_camera_coords)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL 1.11.1)

add_definitions(${PCL_DEFINITIONS})
add_subdirectory(subdirectories/eigen)
add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/subdirectories/eigen)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} eigen)

