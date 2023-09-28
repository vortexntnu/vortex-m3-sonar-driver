cmake_minimum_required(VERSION 3.5)
project(vortex_m3_sonar_driver)

# Find dependencies
find_package(ament_cmake     REQUIRED)
find_package(librapidxml-dev REQUIRED)

# Specify the include directories
include_directories(include)

# Create executable
add_executable(${PROJECT_NAME}_main
    src/main.cpp
    src/XmlCommandGenerator.cpp
    src/XmlResponseParser.cpp
)
ament_target_dependencies(${PROJECT_NAME}_main
    librapidxml-dev
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

# # Create a shared library (optional, only if needed)
# add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

# # Install headers and library (optional)
# ament_target_dependencies(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test 
    test/main.cpp
    test/XmlCommandGeneratorTest.cpp
    test/XmlResponseParserTest.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    librapidxml-dev
  )

endif()

ament_package()
