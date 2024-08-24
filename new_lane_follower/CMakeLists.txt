cmake_minimum_required(VERSION 3.8)
project(new_lane_follower LANGUAGES C CXX CUDA)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CUDA_MIN_VERSION "10.2")


execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(Armadillo REQUIRED)
find_package(OpenMP REQUIRED)
find_package(mlpack REQUIRED)
find_package(Boost REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${mlpack_INCLUDE_DIRS}
  ${armadillo_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
# ament_python_install_package(${PROJECT_NAME})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(DBSCAN src/dbscan_cuda.cu)

add_executable(lane_follower src/lane_follower.cpp)
target_link_libraries(lane_follower
  ${OpenCV_LIBRARIES}
  ${mlpack_LIBRARIES}
  ${armadillo_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CUDA_LIBRARIES}
  DBSCAN
)
ament_target_dependencies(lane_follower rclcpp sensor_msgs cv_bridge OpenCV Armadillo OpenMP mlpack Boost std_msgs sensor_msgs pcl_ros pcl_conversions)

add_executable(one_lane_follower src/one_lane_follower.cpp)
target_link_libraries(one_lane_follower
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(one_lane_follower rclcpp sensor_msgs cv_bridge OpenCV)
# add_executable(lane_follower_2 src/lane_follower_2.cpp)
# target_link_libraries(lane_follower_2
#   ${OpenCV_LIBRARIES}
# )
# ament_target_dependencies(lane_follower_2 rclcpp sensor_msgs cv_bridge OpenCV std_msgs sensor_msgs)


add_executable(ipm src/ipm.cu)
# set_property(TARGET ipm PROPERTY CUDA_ARCHITECTURES "87")
target_link_libraries(ipm 
  ${OpenCV_LIBRARIES}
  ${mlpack_LIBRARIES}
  ${armadillo_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(ipm
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
)

 install(
   PROGRAMS 
     src/intensity.py
     src/go.py
     src/go_new.py
     src/robot_navigator.py
     src/hsv.py
     src/hsv_thresholding.py
     src/multi-otsu.py
   DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  one_lane_follower
  lane_follower
  ipm
  
  # lane_follower_2
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  ${PCL_INCLUDE_DIRS}
)

ament_package()
