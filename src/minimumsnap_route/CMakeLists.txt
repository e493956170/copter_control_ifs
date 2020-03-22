cmake_minimum_required(VERSION 2.8.3)
project(minimumsnap_route)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs 
  std_msgs  
  nav_msgs 
  tf
  image_transport
  cv_bridge
  mavros_msgs
  message_generation
) 
 
# find_package(OSQP REQUIRED)

# find_package(VTK REQUIRED)
# include(${VTK_USE_FILE})

add_service_files(
  FILES
  service.srv
)


generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)



