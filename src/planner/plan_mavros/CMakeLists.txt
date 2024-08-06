cmake_minimum_required(VERSION 3.0.2)
project(plan_mavros)

find_package(catkin REQUIRED COMPONENTS
  mavros
  quadrotor_msgs
  roscpp
  tf
  serial
)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plan_mavros
#  CATKIN_DEPENDS mavros plan_manage roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include/plan_mavros
  ${catkin_INCLUDE_DIRS}
)


add_executable(drop_mainner_node
  src/drop_mainner.cpp 
  src/mavros_manager.cpp
  )
target_link_libraries(drop_mainner_node 
  ${catkin_LIBRARIES}
  )
add_executable(test2 src/test.cpp)
target_link_libraries(test2 
  ${catkin_LIBRARIES}
  )
 
