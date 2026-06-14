cmake_minimum_required(VERSION 3.16) # minimum CMake version we rely on
project(KalmanTracker CXX) # project name

set(CMAKE_CXX_STANDARD 17) # compile against C++17 standard
set(CMAKE_CXX_STANDARD_REQUIRED ON) # fail if the compiler can't do C++17

# Point straight at the extracted Eigen headers
set(EIGEN_DIR "C:/libs/eigen-5.0.0")

# Define the program tracker, built from these two source files
add_executable(tracker src/main.cpp src/KalmanFilter.cpp)

# Tell the compiler our headers live in include/
target_include_directories(tracker PRIVATE include ${EIGEN_DIR})