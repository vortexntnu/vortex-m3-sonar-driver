add_library(m3_driver
  imb_data_listener.cpp
  imb_data_publisher.cpp
  imb_parser.cpp
)

target_include_directories(m3_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(m3_driver PUBLIC
  Eigen3::Eigen
  spdlog::spdlog
  pcl_common
  ${PCL_LIBRARIES}
)

install(TARGETS m3_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
