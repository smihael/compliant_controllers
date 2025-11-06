cmake_minimum_required(VERSION 3.5)
project(compliant_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-return-type)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(compliant_controllers_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Robot model implementation library (encapsulates pinocchio usage)
add_library(robot_model_impl SHARED
  src/robot_model.cpp
)

target_include_directories(robot_model_impl PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(robot_model_impl
  pinocchio
)

add_library(cartesian_impedance_impl SHARED
  src/cartesian_impedance_impl.cpp
)

target_include_directories(cartesian_impedance_impl PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Keep this implementation library minimal; only link required internal and math deps.
target_link_libraries(cartesian_impedance_impl
  PUBLIC robot_model_impl
)
if(UNIX)
  target_link_libraries(cartesian_impedance_impl PRIVATE dl)
endif()

add_library(${PROJECT_NAME} SHARED
  src/generic_cartesian_controller_wrapper.cpp 
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  franka_msgs
  franka_semantic_components
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  sensor_msgs
  compliant_controllers_msgs
  pinocchio # transitively for interface if needed (Jacobian types)
  ament_index_cpp
)

# Standalone implementation library exposing factory symbols for dynamic loading

# Link wrapper with implementation libs
target_link_libraries(${PROJECT_NAME} cartesian_impedance_impl robot_model_impl)
if(UNIX)
  target_link_libraries(${PROJECT_NAME} dl)
endif()

pluginlib_export_plugin_description_file(controller_interface compliant_controllers.xml)

install(TARGETS robot_model_impl cartesian_impedance_impl ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES compliant_controllers.xml
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/test_cartesian_command.py
  scripts/test_cartesian_directional_impedance.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY
    launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} cartesian_impedance_impl robot_model_impl)
ament_export_dependencies(controller_interface pluginlib rclcpp rclcpp_lifecycle hardware_interface pinocchio ament_index_cpp)

ament_package()
