cmake_minimum_required(VERSION 3.5)
project(vortex-m3-sonar-driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fopenmp)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Specify the include directories
include_directories(include)

# Create executable
add_executable(${PROJECT_NAME}_main
    src/main.cpp
    src/XmlCommandGenerator.cpp
    src/XmlResponseParser.cpp
)
ament_target_dependencies(${PROJECT_NAME}_main
)

# Install the headers
install(
    DIRECTORY include/
    DESTINATION include
)

# Install the library
install(TARGETS 
    ${PROJECT_NAME}_main
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test 
    test/main.cpp
    test/XmlCommandGeneratorTest.cpp
    test/XmlResponseParserTest.cpp

    src/XmlCommandGenerator.cpp
    src/XmlResponseParser.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
  )

endif()

ament_package()
