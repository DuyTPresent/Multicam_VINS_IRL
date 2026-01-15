cmake_minimum_required(VERSION 2.8.3)
project(multicam_vins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++1z")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  camera_model
  sensor_msgs
  )

find_package(Gflags)
find_package(Glog)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(TBB REQUIRED)


include_directories(thirdparty/Sophus)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS camera_model)

add_executable(multicam_estimator
  
  src/Frontend/feature_tracker/trackerbase.cpp
  src/Frontend/feature_tracker/image_processing.cpp
  src/Frontend/feature_tracker/ORBextractor/ORBextractor.cc
  src/Frontend/Multicam_VINS_frontend.cpp

  src/Estimator/Multicam_VINS_estimator.cpp
  src/Estimator/Multicam_VINS_feature_manager.cpp
  src/Estimator/parameters.cpp

  src/Estimator/factor/pose_local_parameterization.cpp
  src/Estimator/factor/projection_factor.cpp
  src/Estimator/factor/projection_td_factor.cpp
  src/Estimator/factor/marginalization_factor.cpp
  src/Estimator/factor/SE3AbsolutatePoseFactor.cpp

  src/utility/utility.cpp
  src/utility/visualization.cpp
  src/utility/CameraPoseVisualization.cpp

  src/Estimator/initial/solve_5pts.cpp
  src/Estimator/initial/initial_sfm.cpp
  src/Estimator/initial/initial_aligment.cpp
  src/Estimator/initial/initial_ex_rotation.cpp

  src/Multicam_VINS_node.cpp
)

add_dependencies(multicam_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(multicam_estimator
  ${catkin_LIBRARIES}
  ${TBB_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  )


