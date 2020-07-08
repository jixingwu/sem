cmake_minimum_required(VERSION 2.8.3)
project(package_RGB)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        cv_bridge
        #camera_models
        image_transport
        )

find_package(OpenCV REQUIRED)


find_package(Eigen3)
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS
        LIBRARIES
        std_msgs
        geometry_msgs
        nav_msgs
        #camera_models
        image_transport
#        darknet_ros_msgs
        tf
        cv_bridge

)

add_executable(package_test src/main.cpp)
target_link_libraries(package_test  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Eigen_LIBS})
