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

# Create executable
# add_executable(${PROJECT_NAME}_main
#     src/main.cpp
#     src/api/m3CommandGenerator.cpp
#     src/api/m3ResponseParser.cpp
#     src/utils/netUtils.cpp
    
# )
add_executable(M3Listener
  src/imb/M3Listener.cpp
  src/imb/imbFormat.cpp
  src/imb/M3Publisher.cpp
)

# ament_target_dependencies(${PROJECT_NAME}_main
#     Eigen3
# )

ament_target_dependencies(M3Listener
    pcl_conversions
    Eigen3
    rclcpp
    sensor_msgs
)
# target_link_libraries(${PROJECT_NAME}_main 
#   PUBLIC Eigen3::Eigen
# )


# Install the headers
install(
    DIRECTORY include
    DESTINATION include
)

# Install the library
install(TARGETS 
    M3Listener
    # ${PROJECT_NAME}_main
    DESTINATION lib/${PROJECT_NAME}
)
target_link_libraries(M3Listener
  pcl_common
  ${PCL_LIBRARIES}
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_test 
#     # test/main.cpp
#     # test/XmlCommandGeneratorTest.cpp
#     # test/XmlResponseParserTest.cpp
#     # test/ImbFormatTest.cpp
#     # test/netUtilsTest.cpp
#     # test/tcpTest.cpp
#     # test/M3SonarTest.cpp
#     # src/api/m3CommandGenerator.cpp
#     # src/api/m3ResponseParser.cpp
#     # src/imb/imbFormat.cpp
#     # src/utils/netUtils.cpp
#     # src/imb/TcpServer.cpp
#   )
#   target_include_directories(${PROJECT_NAME}_test PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   ament_target_dependencies(${PROJECT_NAME}_test
#     Eigen3
#   )

# endif()

ament_package()
