cmake_minimum_required(VERSION 3.21)
project(chhRobotics_CPP)

set(CMAKE_CXX_STANDARD 14)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# bezier
add_executable(bezier_demo Bezier/main.cpp Bezier/BezierCurve.cpp)
target_link_libraries(bezier_demo PRIVATE chhRobotics_CPP)

# B-spline
add_executable(b_spline_demo B-spline/main.cpp B-spline/BSpline.cpp)
target_link_libraries(b_spline_demo PRIVATE chhRobotics_CPP)

# 曲线插值
add_executable(curve_demo Curve_interpolation/main.cpp )
target_link_libraries(curve_demo PRIVATE chhRobotics_CPP)


# 人势场法
add_executable(apf_demo Artifical_Potential_Field/main.cpp Artifical_Potential_Field/APF.cpp)
target_link_libraries(apf_demo PRIVATE chhRobotics_CPP)