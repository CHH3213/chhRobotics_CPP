cmake_minimum_required(VERSION 3.21)
project(chhRobotics_CPP)

set(CMAKE_CXX_STANDARD 14)

#PID
add_executable(pid_demo PID/main.cpp PID/PID_controller.cpp PID/KinematicModel.cpp)
target_link_libraries(pid_demo PRIVATE chhRobotics_CPP)

#pure pursuit
add_executable(pure_pursuit_demo Pure_Pursuit/main.cpp  Pure_Pursuit/PurePursuit.cpp PID/KinematicModel.cpp)
target_link_libraries(pure_pursuit_demo PRIVATE chhRobotics_CPP)

#stanley
add_executable(stanley_demo Stanley/main.cpp  Stanley/Stanley.cpp PID/KinematicModel.cpp)
target_link_libraries(stanley_demo PRIVATE chhRobotics_CPP)


# 后轮位置反馈控制
add_executable(rear_wheel_feedback_demo Rear_Wheel_Feedback/main.cpp  Rear_Wheel_Feedback/RearWheelFeedback.cpp Rear_Wheel_Feedback/MyReferencePath.cpp PID/KinematicModel.cpp)
target_link_libraries(rear_wheel_feedback_demo PRIVATE chhRobotics_CPP)

