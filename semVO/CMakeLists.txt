cmake_minimum_required(VERSION 2.8.3)
project(cube_vo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
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
    darknet_ros_msgs
        message_filters
        )

find_package(OpenCV REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

find_package(Sophus REQUIRED)
set(Sophus_LIBRARIES libSophus.so)
include_directories(${Sophus_INCLUDE_DIRS})

#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS
        LIBRARIES
        std_msgs
        geometry_msgs
        nav_msgs
        #camera_models
        image_transport
        darknet_ros_msgs
        tf
        cv_bridge
        message_filters

)

add_library(cube_vo_lib
        src/detect_3d_cuboid/box_proposal_detail.cpp
        src/detect_3d_cuboid/matrix_utils.cpp
        src/detect_3d_cuboid/object_3d_util.cpp
        src/line_lbd/binary_descriptor.cpp
        src/line_lbd/binary_descriptor_matcher.cpp
        src/line_lbd/draw.cpp
        src/line_lbd/line_lbd_allclass.cpp
        src/line_lbd/lsd.cpp
        src/line_lbd/LSDDetector.cpp
        src/TopoMetric.cpp

        #vo
        src/camera.cpp
        src/config.cpp
        src/frame.cpp
        src/map.cpp
        src/mapcube.cpp
        src/visual_odometry.cpp
        src/visualization.cpp)

target_link_libraries(cube_vo_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${G2O_LIBRARIES} ${Sophus_LIBRARIES})


#add_executable(vins_node src/rosNodeTest.cpp)
#target_link_libraries(vins_node vins_lib)
#
#add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
#target_link_libraries(kitti_odom_test vins_lib)
#
#add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
#target_link_libraries(kitti_gps_test vins_lib)

add_executable(cube_vo_test src/cube_vo_node.cpp)
target_link_libraries(cube_vo_test cube_vo_lib ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)

