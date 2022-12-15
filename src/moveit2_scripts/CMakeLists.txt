cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(test_trajectory2
  src/test_trajectory2.cpp)





# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

ament_target_dependencies(test_trajectory2 moveit_core ) 
ament_target_dependencies(test_trajectory2 rviz_visual_tools ) 
ament_target_dependencies(test_trajectory2 moveit_ros_planning ) 
ament_target_dependencies(test_trajectory2 moveit_ros_planning_interface ) 

target_include_directories(test_trajectory2
  PUBLIC include)

install(TARGETS 
	test_trajectory2
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
