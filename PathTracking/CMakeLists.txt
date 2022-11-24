cmake_minimum_required(VERSION 3.21)
project(chhRobotics_CPP)

set(CMAKE_CXX_STANDARD 14)

add_executable(pid_demo PID/main.cpp PID/PID_controller.h PID/KinematicModel.h PID/PID_controller.cpp PID/KinematicModel.cpp)
target_link_libraries(pid_demo PRIVATE chhRobotics_CPP)

