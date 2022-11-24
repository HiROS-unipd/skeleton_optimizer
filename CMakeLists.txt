cmake_minimum_required(VERSION 3.5)
project(hiros_skeleton_optimizer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hiros_skeleton_msgs REQUIRED)
find_package(hiros_skeleton_tracker REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(srv_files
  "srv/Calibrate.srv"
  "srv/ChangeId.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files})
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(${PROJECT_NAME}_node
    src/main.cpp
    src/CostFunction.cpp
    src/Optimizer.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp hiros_skeleton_msgs hiros_skeleton_tracker)

set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME
  ${PROJECT_NAME}
  PREFIX
  ""
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE include ${EIGEN_INCLUDE_DIRS} ${CERES_INCUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}" ${CERES_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
