cmake_minimum_required(VERSION 3.8)
project(vda5050_master_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(vda5050_core REQUIRED)

add_executable(test_node
  src/main.cpp
)

target_link_libraries(test_node
  vda5050_core::logger
  vda5050_core::types
  vda5050_core::execution
  vda5050_core::master
)

install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  vda5050_ament_lint_common()
endif()

ament_package()
