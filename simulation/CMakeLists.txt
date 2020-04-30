cmake_minimum_required(VERSION 3.10)

# set the project name
project(adaptive_attitude_controller VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# debug
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Python3 COMPONENTS Development NumPy)

# add include directory
include_directories(include)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

# add executables
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} Python3::Python Python3::NumPy)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} dynamics)

add_library(dynamics src/dynamics/dynamics.cpp)
target_link_libraries(dynamics Eigen3::Eigen)

add_library(attitude_controller src/controller/attitude_controller.cpp)
target_link_libraries(attitude_controller Eigen3::Eigen)
