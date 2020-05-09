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

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Python3 COMPONENTS Development NumPy)

find_package(gflags REQUIRED) # Google flags
include_directories(${gflags_INCLUDE_DIRS})

list(APPEND CMAKE_PREFIX_PATH /Users/bernhardpausgraesdal/google_drive/Studier/8th_semester/6_832_underactuated_robotics/04_underactuated_project/drake/build/install)
find_package(drake CONFIG REQUIRED)
include_directories(${drake_INCLUDE_DIRS})

#list(APPEND CMAKE_PREFIX_PATH /Users/bernhardpausgraesdal/google_drive/Studier/8th_semester/6_832_underactuated_robotics/04_underactuated_project/iris-distro/build/install)

# Installing IRIS:
#		1. Clone github repo https://github.com/rdeits/iris-distro
#		2. mkdir build
#		3. cd build
#		4. make
# Make sure to have Mosek installed, with a valid license
# The following two lines are required to include IRIS
#find_package(iris REQUIRED)
link_directories(/Users/bernhardpausgraesdal/google_drive/Studier/8th_semester/6_832_underactuated_robotics/04_underactuated_project/iris-distro/build/install/lib)
include_directories(/Users/bernhardpausgraesdal/google_drive/Studier/8th_semester/6_832_underactuated_robotics/04_underactuated_project/iris-distro/build/install/include)

# add include directory
include_directories(include)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

# add executables
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} drake::drake)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} Python3::Python)
target_link_libraries(${PROJECT_NAME} Python3::NumPy)
target_link_libraries(${PROJECT_NAME} iris)
target_link_libraries(${PROJECT_NAME} tests)
target_link_libraries(${PROJECT_NAME} trajopt)
target_link_libraries(${PROJECT_NAME} simulate)

add_library(trajopt src/trajopt/MISOSProblem.cpp src/trajopt/PPTrajectory.cpp)
target_link_libraries(trajopt drake::drake)
target_link_libraries(trajopt Eigen3::Eigen)

add_library(plotter src/plot/plotter.cpp)
target_link_libraries(plotter Eigen3::Eigen)
target_link_libraries(plotter trajopt)
target_link_libraries(plotter convex_hull)

add_library(convex_hull src/tools/convexHull.cpp)
target_link_libraries(convex_hull Eigen3::Eigen)

add_library(tests src/test/tests.cpp)
target_link_libraries(tests Eigen3::Eigen)
target_link_libraries(tests trajopt)
target_link_libraries(tests plotter)

add_library(simulate src/simulate/simulate.cpp)
target_link_libraries(simulate drake::drake)
target_link_libraries(simulate Eigen3::Eigen)
target_link_libraries(simulate gflags)
target_link_libraries(simulate geometry)

add_library(geometry src/tools/geometry.cpp)
target_link_libraries(geometry drake::drake)
