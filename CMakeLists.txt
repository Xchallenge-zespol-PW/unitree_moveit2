cmake_minimum_required(VERSION 3.8)
project(unitree_moveit2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Plik wykonywalny łączący main oraz obie klasy pomocnicze
add_executable(unitree_node 
  src/logic_node_test.cpp 
  src/arm_controller.cpp 
  src/collision_manager.cpp
)

target_include_directories(unitree_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(unitree_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  unitree_node
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "shape_msgs"
  "moveit_msgs"
  "tf2"
  "tf2_geometry_msgs"
)

install(TARGETS unitree_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()