cmake_minimum_required(VERSION 3.10)

# set the project name
project(mixed_integer_planning VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# debug
set(CMAKE_BUILD_TYPE Debug)


# find drake
find_package(
	drake CONFIG REQUIRED
	)

# Installing IRIS:
#		1. Clone github repo https://github.com/rdeits/iris-distro
#		2. mkdir build
#		3. cd build
#		4. make
# Make sure to have Mosek installed, with a valid license
# The following two lines are required to include IRIS
link_directories(/Users/bernhardpausgraesdal/google_drive/Studier/8th_semester/6_832_underactuated_robotics/04_underactuated_project/iris-distro/build/install/lib)
include_directories(/Users/bernhardpausgraesdal/google_drive/Studier/8th_semester/6_832_underactuated_robotics/04_underactuated_project/iris-distro/build/install/include)

find_package(Python3 COMPONENTS Development NumPy)


# add include directory
include_directories(include)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

# add the executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} drake::drake trajopt)
target_link_libraries(${PROJECT_NAME} iris trajopt)
target_link_libraries(${PROJECT_NAME} Python3::Python Python3::NumPy)

add_library(trajopt src/trajectory_optimization.cpp)
add_library(convex_hull src/tools/convexHull.cpp)
target_link_libraries(convex_hull drake::drake)
target_link_libraries(trajopt drake::drake)
target_link_libraries(trajopt convex_hull)


