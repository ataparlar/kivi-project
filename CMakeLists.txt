cmake_minimum_required(VERSION 3.5.0)
project(kivi_project VERSION 0.1.0 LANGUAGES C CXX)

find_package(OpenCV REQUIRED)

add_executable(kivi_project 
    src/main.cpp 
    src/phase_map_generator.cpp
    src/3d_bilateral_filtering.cpp)

include_directories(include)

target_link_libraries(kivi_project ${OpenCV_LIBS})

include(CTest)
enable_testing()

