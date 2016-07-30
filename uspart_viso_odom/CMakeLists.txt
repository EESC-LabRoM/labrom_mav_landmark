cmake_minimum_required(VERSION 2.8.3)
project(uspart_viso_odom)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  cv_bridge
  image_transport
  roscpp

)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME} 
   CATKIN_DEPENDS geometry_msgs sensor_msgs nav_msgs tf cv_bridge image_transport roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)

## Declare a C++ library
set(${PROJECT_NAME}_LIB_SOURCES
    src/monocular_node.cpp
    src/monocular_odom.cpp
    nodes/vo_mono_example.cpp
)

add_library(${PROJECT_NAME} 
       ${${PROJECT_NAME}_LIB_SOURCES} )

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable( vo_mono_example nodes/vo_mono_example.cpp  )
target_link_libraries( vo_mono_example ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBS} )
