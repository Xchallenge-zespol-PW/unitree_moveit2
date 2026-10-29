cmake_minimum_required(VERSION 3.8)
project(unitree_arms)

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
find_package(tf2_ros REQUIRED) # <-- DODANO: Wymagane dla transformacji TF2

# 1. Tworzymy bibliotekę z logiki robota (klasy pomocnicze)
add_library(unitree_arms_core
  src/arm_controller.cpp 
  src/collision_manager.cpp
  src/unitree_moveit_interface.cpp
)

target_include_directories(unitree_arms_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(unitree_arms_core PUBLIC cxx_std_17)

ament_target_dependencies(
  unitree_arms_core
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "shape_msgs"
  "moveit_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

# 2. Tworzymy plik wykonywalny (węzeł testowy / główny)
add_executable(logic_node_test src/logic_node_test.cpp)
target_compile_features(logic_node_test PUBLIC cxx_std_17)

ament_target_dependencies(
  logic_node_test
  "rclcpp"
  "geometry_msgs"
)

# Podpinamy naszą bibliotekę do pliku wykonywalnego
target_link_libraries(logic_node_test unitree_arms_core)

# Instalacja
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  unitree_arms_core
  logic_node_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()