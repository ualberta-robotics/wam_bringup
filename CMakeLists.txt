cmake_minimum_required(VERSION 2.8.3)
project(wam_bringup)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${Barrett_INCLUDE_DIRS})
link_directories(${Barrett_LIBRARY_DIRS})
add_definitions(${Barrett_DEFINITIONS})

## GSL
set(GSL_INCLUDE_DIRS "/usr/include/gsl")
set(GSL_LIBRARY_DIRS "/usr/lib")
set(GSL_LIBRARY "/usr/lib/libgsl.so.0")

#find_package(gsl REQUIRED)
include_directories(${GSL_INCLUDE_DIRS})
link_directories(${GSL_LIBRARY_DIRS})

## Curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIRS})

## Boost
find_package(Boost 1.46 COMPONENTS system thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

## Eigen2
#find_package(Eigen REQUIRED)
set(Eigen_INCLUDE_DIRS "/usr/include/eigen2")
include_directories(${Eigen_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
  wam_msgs
  wam_srvs
)


## xenomai
#find_package(Xenomai REQUIRED)
set(XENOMAI_INCLUDE_XENOMAI "/usr/xenomai/include")
set(XENOMAI_INCLUDE_GENERIC "/usr/src/linux-2.6.38/include")
set(XENOMAI_XENOMAI "/usr/xenomai/lib/libxenomai.so.0")
set(XENOMAI_NATIVE "/usr/xenomai/lib/libnative.so.3")
set(XENOMAI_LIBRARY_DIRS "/usr/xenomai/lib")
set(XENOMAI_INCLUDE_DIRS ${XENOMAI_INCLUDE_XENOMAI} ${XENOMAI_INCLUDE_GENERIC})
set(XENOMAI_LIBRARIES ${XENOMAI_XENOMAI} ${XENOMAI_NATIVE})

include_directories(${XENOMAI_INCLUDE_DIRS})
link_directories(${XENOMAI_LIBRARY_DIRS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES wam_node
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs std_srvs tf wam_msgs wam_srvs
#  DEPENDS system_lib
)

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${BARRETT_INCLUDE_DIRS}
#  ${Barrett_INCLUDE_DIRS}
#  include
#)

add_executable(wam_node src/wam_node.cpp)
add_dependencies(wam_node wam_node_generate_messages_cpp)
target_link_libraries(
  wam_node
  barrett 
  ${catkin_LIBRARIES} 
  ${CURSES_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GSL_LIBRARY}
  ${XENOMAI_LIBRARIES}
  )
add_executable(bhand_node src/bhand_node.cpp)
add_dependencies(bhand_node bhand_node_generate_messages_cpp)
target_link_libraries(
  bhand_node
  barrett 
  ${catkin_LIBRARIES} 
  ${CURSES_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GSL_LIBRARY}
  ${XENOMAI_LIBRARIES}
  )

#add_executable(wam_bringup src/wam_bringup.cpp)
#add_dependencies(wam_bringup wam_bringup_generate_messages_cpp)
#target_link_libraries(
#  wam_bringup
#  barrett 
#  ${catkin_LIBRARIES} 
#  ${CURSES_LIBRARIES}
#  ${Boost_LIBRARIES}
#  ${GSL_LIBRARY}
#  ${XENOMAI_LIBRARIES}
#  )

#add_executable(velocity_control src/cartesian_velocity_control.cpp)
#add_dependencies(velocity_control wam_bringup_generate_messages_cpp)
#target_link_libraries(
#  velocity_control
#  barrett 
#  ${catkin_LIBRARIES} 
#  ${CURSES_LIBRARIES}
#  ${Boost_LIBRARIES}
#  ${GSL_LIBRARY}
#  ${XENOMAI_LIBRARIES}
#  )

