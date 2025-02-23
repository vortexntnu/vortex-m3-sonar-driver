add_subdirectory(m3_driver)

add_executable(m3_driver_node ros_m3_driver/ros_node.cpp)

target_include_directories(m3_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/m3_driver>
  $<INSTALL_INTERFACE:include/m3_driver>
)

ament_target_dependencies(m3_driver_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(m3_driver_node
  m3_driver
  spdlog::spdlog
  pcl_common
  ${PCL_LIBRARIES}
)

install(TARGETS m3_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
