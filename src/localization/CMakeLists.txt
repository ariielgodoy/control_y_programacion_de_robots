cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wunused-parameter)
endif() 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
# For ParticleFilter
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
# For srv
find_package(rosidl_default_generators REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare services here!
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetEstimation.srv"
  DEPENDENCIES geometry_msgs
)

# Includes
include_directories(
  include
  $ENV{EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Install launch files and related config files.
install(DIRECTORY
  config
  launch
  scenes
  DESTINATION share/${PROJECT_NAME}
)

# Add Executables
add_executable(lse src/lse.cpp)
add_executable(pf src/pf.cpp)

# Dependencies
ament_target_dependencies(lse rclcpp std_msgs geometry_msgs Eigen3)
ament_target_dependencies(pf rclcpp std_msgs geometry_msgs Eigen3 pcl_conversions)  #tf2

# Declare srv support for nodes
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lse "${cpp_typesupport_target}")
target_link_libraries(pf "${cpp_typesupport_target}")

# Declare and install executables
install(TARGETS
  lse
  pf
  DESTINATION lib/${PROJECT_NAME}
)

# Pkg
ament_package()