cmake_minimum_required(VERSION 3.8)
project(flight_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(landing_test src/landing_test.cpp)
add_executable(landing_test_vel src/landing_test_vel.cpp)
add_executable(landing_test_flight src/landing_test_flight.cpp)

ament_target_dependencies(landing_test rclcpp px4_msgs geometry_msgs)
target_include_directories(landing_test PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(landing_test_vel rclcpp px4_msgs geometry_msgs)
target_include_directories(landing_test_vel PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(landing_test_flight rclcpp px4_msgs geometry_msgs)
target_include_directories(landing_test_flight PUBLIC ${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  landing_test
  landing_test_vel
  landing_test_flight
  DESTINATION lib/${PROJECT_NAME})

ament_package()
