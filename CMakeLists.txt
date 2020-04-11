cmake_minimum_required(VERSION 3.10)

# set the project name
project(mixed_integer_planning VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)

# add the executable
add_executable(${PROJECT_NAME} src/main.cpp)
