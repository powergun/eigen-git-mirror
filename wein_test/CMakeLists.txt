cmake_minimum_required(VERSION 3.8)
project(weintest)

add_executable(weintest main.cpp)
set_target_properties(weintest
    PROPERTIES
    CXX_STANDARD 14)
target_include_directories(weintest
    PUBLIC
    ../)

