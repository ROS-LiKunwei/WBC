cmake_minimum_required(VERSION 3.8)
project(ik_7dof)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. 创建库 (IK 解算器)
add_library(ik_solver_lib
  src/ik_solver.cpp
)
add_library(fa_ik_solver_lib
  src/fa_ik_solver.cpp
)

# 使用 ament 的方式处理依赖，这会自动处理头文件和库
ament_target_dependencies(ik_solver_lib
  Eigen3
  pinocchio
)
ament_target_dependencies(fa_ik_solver_lib
  Eigen3
  pinocchio
)

target_include_directories(ik_solver_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
target_include_directories(fa_ik_solver_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

# 3. 创建可执行文件 (ROS 2 节点)
add_executable(left_arm_ik_node
  src/left_arm_ik_node.cpp
)
add_executable(fa_left_arm_ik_node
  src/fa_left_arm_ik_node.cpp
)
add_executable(arm_ik_rviz_node
  src/arm_ik_rviz_node.cpp
)

# 让节点链接到库并获取所有 ROS 2 依赖环境
ament_target_dependencies(left_arm_ik_node
  rclcpp
  Eigen3
  pinocchio
)
ament_target_dependencies(fa_left_arm_ik_node
  rclcpp
  Eigen3
  pinocchio
)

ament_target_dependencies(arm_ik_rviz_node
  rclcpp
  Eigen3
  pinocchio
  tf2_ros
  geometry_msgs
  sensor_msgs
)

# 链接你自己的库
target_link_libraries(left_arm_ik_node
  ik_solver_lib
)
target_link_libraries(fa_left_arm_ik_node
  fa_ik_solver_lib
)
target_link_libraries(arm_ik_rviz_node
  fa_ik_solver_lib
)

# 4. 安装
install(
  TARGETS
    ik_solver_lib
    fa_ik_solver_lib
    left_arm_ik_node
    fa_left_arm_ik_node
    arm_ik_rviz_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY  launch 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()