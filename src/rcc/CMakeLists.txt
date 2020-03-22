cmake_minimum_required(VERSION 2.8.3)
project(rcc)

## Compile as C++11, supported in ROS Kinetic and newer
SET(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
add_compile_options(-fopenmp)
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
# SET(CMAKE_C_FLAGS "$ENV{CFLAGS} -O3 -Wall")
# SET(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3")  

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros 
  sensor_msgs
  std_msgs
  nav_msgs 
  tf 
  image_transport
  cv_bridge
  mavros_msgs
  octomap_msgs
)  
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
 
find_package(OpenCV REQUIRED)

generate_messages(DEPENDENCIES std_msgs) 

catkin_package(  
  CATKIN_DEPENDS message_runtime 
) 
include_directories( 
  ${catkin_INCLUDE_DIRS}
) 
include_directories(
  ../../devel/include 
)
aux_source_directory(src DIR_SRCS)  ##查找目录下的所有文,然后存入变量 DIR_SRCS
aux_source_directory(src/flight_task flight_task_SRCS)  ##查找目录下的所有文,然后存入变量 DIR_SRCS

include_directories(include)  
include_directories(mavlink)
message(STATUS "    源文件列表：${DIR_SRCS}")
ADD_EXECUTABLE(rcc ${DIR_SRCS} ${flight_task_SRCS})
FIND_PACKAGE(PCL REQUIRED COMPONENTS visualization )

target_link_libraries(rcc  
${catkin_LIBRARIES} 
${PCL_LIBRARIES}  
${OpenCV_LIBS}  
${OCTOMAP_LIBRARIES}
)

