cmake_minimum_required(VERSION 3.8)
project(closetbot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(fp REQUIRED)
find_package(range-v3 REQUIRED)
find_package(serial REQUIRED)

add_library(${PROJECT_NAME}
  src/command.cpp
  src/detect.cpp
  src/closetbot.cpp
  src/process_received.cpp
  src/serialize.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)  # C++20  
  
  
add_executable(my_node src/my_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    fmt::fmt
    range-v3::range-v3
  PUBLIC
    fp::fp
    serial::serial
)

add_executable(wiggle bringup/wiggle.cpp)
target_link_libraries(wiggle
  ${PROJECT_NAME}
  fp::fp
)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME} wiggle
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
