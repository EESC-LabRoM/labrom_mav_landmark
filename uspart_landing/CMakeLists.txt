cmake_minimum_required(VERSION 2.8.3)
project(uspart_landing)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  uspart_viso_odom
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
   CATKIN_DEPENDS  geometry_msgs nav_msgs roscpp sensor_msgs tf uspart_viso_odom 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)

## Declare a C++ library
set(${PROJECT_NAME}_LIB_SOURCES
    src/descriptor.cpp
    src/landmark.cpp
)

add_library(${PROJECT_NAME} 
       ${${PROJECT_NAME}_LIB_SOURCES} )

add_executable( learn nodes/learning.cpp  )
target_link_libraries( learn ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})

add_executable( generic nodes/generic.cpp  )
target_link_libraries( generic ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})

add_executable( circular nodes/circular.cpp  )
target_link_libraries( circular ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})






