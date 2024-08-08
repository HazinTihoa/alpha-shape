cmake_minimum_required(VERSION 3.10)
project(PointCloudBoundaryDetection)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找所需的库

find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

# 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 链接库
link_directories(${PCL_LIBRARY_DIRS})

# 编译标志
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(PointCloudBoundaryDetection src/ashape.cpp)

# 链接库
target_link_libraries(PointCloudBoundaryDetection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cgal-ashape src/cgal-ashape.cpp)
target_link_libraries(cgal-ashape ${PCL_LIBRARIES} ${CGAL_LIBRARIES})
