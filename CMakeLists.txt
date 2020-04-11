cmake_minimum_required(VERSION 3.10)

# set the project name
project(mixed_integer_planning VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find drake
find_package(
	drake CONFIG REQUIRED
	)

# add include directory
include_directories(include)

# add the executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} drake::drake trajopt)

add_library(trajopt src/trajectory_optimization.cpp)
target_link_libraries(trajopt drake::drake)
