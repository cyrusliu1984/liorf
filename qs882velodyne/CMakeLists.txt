cmake_minimum_required(VERSION 3.10)
project(qs882velodyne)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找catkin依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

# 声明catkin包
catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs
)

# 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 修正：可执行文件名改为全小写（避免大小写问题）
add_executable(qs882velodyne_node src/qs882velodyne.cpp)
target_link_libraries(qs882velodyne_node
  ${catkin_LIBRARIES}
)

install(TARGETS qs882velodyne_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件（确保launch目录存在）
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

