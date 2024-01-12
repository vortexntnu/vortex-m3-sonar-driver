cmake_minimum_required(VERSION 3.5)
project(vortex-m3-sonar-driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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

# Specify the include directories
include_directories(include)

add_executable(M3Publisher
  src/imb/ImbFormat.cpp
  src/imb/M3Listener.cpp
  src/imb/M3Publisher.cpp
)

ament_target_dependencies(M3Publisher
    pcl_conversions
    Eigen3
    rclcpp
    sensor_msgs
)

# Install the headers
install(
    DIRECTORY include
    DESTINATION include
)

install(
    DIRECTORY test/data
    DESTINATION data
)

# Install the library
install(TARGETS 
    M3Publisher
    DESTINATION lib/${PROJECT_NAME}
)
target_link_libraries(M3Publisher
  pcl_common
  ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test 
    test/ImbFormatTest.cpp
    src/imb/ImbFormat.cpp
    
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    pcl_conversions
    Eigen3
    rclcpp
    sensor_msgs
  )
  target_link_libraries(${PROJECT_NAME}_test
    pcl_common
    ${PCL_LIBRARIES}
)

endif()

ament_package()
