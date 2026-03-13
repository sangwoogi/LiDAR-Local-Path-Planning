cmake_minimum_required(VERSION 3.8)
project(nuscenes_data_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. 의존성 패키지 찾기 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# --- 2. 라이다 데이터 퍼블리셔 노드 빌드 ---
add_executable(publisher_node src/nuscenes_publisher.cpp)
target_include_directories(publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(publisher_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(publisher_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(publisher_node ${PCL_LIBRARIES})

# --- 3. 2D 그리드 맵 노드 빌드 ---
add_executable(grid_map_node src/grid_map_node.cpp)
target_include_directories(grid_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(grid_map_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(grid_map_node rclcpp sensor_msgs nav_msgs pcl_conversions)
target_link_libraries(grid_map_node ${PCL_LIBRARIES})

# --- 4. A* 경로 계획 노드 빌드 (신규 추가) ---
add_executable(path_planner_node src/path_planner_node.cpp)
target_include_directories(path_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_planner_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(path_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# --- 5. 설치 경로 지정 ---
install(TARGETS
  publisher_node
  grid_map_node
  path_planner_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()