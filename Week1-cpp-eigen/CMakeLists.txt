cmake_minimum_required(VERSION 3.17)
project(Week1_cpp_eigen)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

# 添加Eigen头文件
include_directories("/usr/include/eigen3")
add_executable(Week1_cpp_eigen main.cpp)