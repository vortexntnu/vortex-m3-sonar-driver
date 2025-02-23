# Root-level CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(vortex-m3-sonar-driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(spdlog REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Add subdirectories
add_subdirectory(src)

# Install headers and data
install(
  DIRECTORY include
  DESTINATION include
)
install(
  DIRECTORY test/data
  DESTINATION data
)

# # Testing configuration
# if(BUILD_TESTING)
#   ament_add_gtest(vortex_m3_sonar_driver_test
#     test/ImbFormatTest.cpp
#     src/m3_driver/imb_parser.cpp
#   )
#   target_include_directories(vortex_m3_sonar_driver_test PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   ament_target_dependencies(vortex_m3_sonar_driver_test
#     pcl_conversions
#     Eigen3
#     rclcpp
#     sensor_msgs
#   )
#   target_link_libraries(vortex_m3_sonar_driver_test
#     pcl_common
#     ${PCL_LIBRARIES}
#   )
# endif()

ament_package()
