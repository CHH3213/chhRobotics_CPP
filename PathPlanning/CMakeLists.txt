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


# 人工势场法
add_executable(apf_demo Artifical_Potential_Field/main.cpp Artifical_Potential_Field/APF.cpp)
target_link_libraries(apf_demo PRIVATE chhRobotics_CPP)

# 动态窗口法
add_executable(dwa_demo Dynamic_Window_Approach/main.cpp Dynamic_Window_Approach/DWA.cpp)
target_link_libraries(dwa_demo PRIVATE chhRobotics_CPP)

# RRT
add_executable(rrt_demo Rapidly-exploring_Random_Tree/main.cpp Rapidly-exploring_Random_Tree/RRT.cpp)
target_link_libraries(rrt_demo PRIVATE chhRobotics_CPP)

# RRT_connect
add_executable(rrt_connect_demo Rapidly-exploring_Random_Tree_connect/main.cpp Rapidly-exploring_Random_Tree_connect/RRT_connect.cpp)
target_link_libraries(rrt_connect_demo PRIVATE chhRobotics_CPP)

# RRT_star
add_executable(rrt_star_demo Rapidly-exploring_Random_Tree_Star/main.cpp Rapidly-exploring_Random_Tree_Star/RRT_Star.cpp Rapidly-exploring_Random_Tree/RRT.cpp)
target_link_libraries(rrt_star_demo PRIVATE chhRobotics_CPP)

# Dijkstra
add_executable(dijkstra_demo Dijkstra/main.cpp Dijkstra/Dijkstra.cpp)
target_link_libraries(dijkstra_demo PRIVATE chhRobotics_CPP)

# astar
add_executable(astar_demo A_Star/Astar.cpp A_Star/main.cpp)
target_link_libraries(astar_demo PRIVATE chhRobotics_CPP)

# RS曲线
add_executable(rs_demo Reeds_Shepp_Path/main.cpp Reeds_Shepp_Path/ReedsShepp.cpp)
target_link_libraries(rs_demo PRIVATE chhRobotics_CPP)

# Dubins曲线
add_executable(dubins_demo Dubins_Path/main.cpp Dubins_Path/Dubins.cpp)
target_link_libraries(dubins_demo PRIVATE chhRobotics_CPP)