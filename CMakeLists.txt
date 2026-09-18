cmake_minimum_required(VERSION 3.5)
project(compliant_controllers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(COMPLIANT_CONTROLLERS_BUILD_TIMING_TESTS "Build standalone controller timing test executables" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-return-type)
  add_compile_options(-O3 -DNDEBUG)
  add_compile_options(-Wno-maybe-uninitialized -Wno-uninitialized)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(compliant_controllers_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ros2_control_robot_dynamics REQUIRED)
find_package(urdf REQUIRED)

add_library(cartesian_impedance_impl SHARED
  src/cartesian_impedance_impl.cpp
)

target_include_directories(cartesian_impedance_impl PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(cartesian_impedance_impl
  ros2_control_robot_dynamics
)
if(UNIX)
  target_link_libraries(cartesian_impedance_impl dl)
endif()

add_library(joint_impedance_impl SHARED
  src/joint_impedance_impl.cpp
)

target_include_directories(joint_impedance_impl PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(joint_impedance_impl
  ros2_control_robot_dynamics
)
if(UNIX)
  target_link_libraries(joint_impedance_impl dl)
endif()

if(COMPLIANT_CONTROLLERS_BUILD_TIMING_TESTS)
  add_executable(joint_update_timing_test
    test/joint_update_timing_test.cpp
  )

  target_include_directories(joint_update_timing_test PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
  )

  target_link_libraries(joint_update_timing_test
    joint_impedance_impl
  )

  ament_target_dependencies(joint_update_timing_test
    ros2_control_robot_dynamics
  )

  add_executable(cartesian_update_timing_test
    test/cartesian_update_timing_test.cpp
  )

  target_include_directories(cartesian_update_timing_test PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
  )

  if(UNIX)
    target_link_libraries(cartesian_update_timing_test dl)
  endif()

  ament_target_dependencies(cartesian_update_timing_test
    ros2_control_robot_dynamics
  )
endif()

add_library(${PROJECT_NAME} SHARED
  src/async_diagnostic_logger.cpp
  src/cartesian_error_biased_dither.cpp
  src/friction_compensation.cpp
  src/generic_controller_wrapper.cpp
  src/generic_cartesian_controller_wrapper.cpp 
  src/generic_joint_controller_wrapper.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  std_msgs
  sensor_msgs
  franka_msgs
  compliant_controllers_msgs
  pinocchio # transitively for interface if needed (Jacobian types)
  ament_index_cpp
  ros2_control_robot_dynamics
  urdf
)

# Standalone implementation library exposing factory symbols for dynamic loading

# Link wrapper with implementation libs
target_link_libraries(${PROJECT_NAME} cartesian_impedance_impl joint_impedance_impl)
if(UNIX)
  target_link_libraries(${PROJECT_NAME} dl)
endif()

pluginlib_export_plugin_description_file(controller_interface compliant_controllers.xml)

install(TARGETS cartesian_impedance_impl joint_impedance_impl ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(COMPLIANT_CONTROLLERS_BUILD_TIMING_TESTS)
  install(TARGETS joint_update_timing_test cartesian_update_timing_test
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(FILES compliant_controllers.xml
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/test_cartesian_command.py
  scripts/test_cartesian_directional_impedance.py
  scripts/test_stiffness_frame_offset.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} cartesian_impedance_impl joint_impedance_impl)
ament_export_dependencies(controller_interface pluginlib rclcpp rclcpp_lifecycle hardware_interface std_msgs sensor_msgs franka_msgs pinocchio ament_index_cpp ros2_control_robot_dynamics urdf)

ament_package()
