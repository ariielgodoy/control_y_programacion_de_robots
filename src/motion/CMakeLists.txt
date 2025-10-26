cmake_minimum_required(VERSION 3.8)
project(motion)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#add_compile_options(-Wall -Wextra -Wunused-parameter)
add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Includes
include_directories(
include
)
# Install launch files and related config files.
install(DIRECTORY
DESTINATION share/${PROJECT_NAME}
)
# Add Executables
add_executable(motion_model src/motion_model.cpp)
add_executable(motion_model_noise src/motion_model_noise.cpp)
add_executable(pf_based_motion src/pf_based_motion.cpp)
# Dependencies
ament_target_dependencies(motion_model rclcpp std_msgs geometry_msgs)
ament_target_dependencies(motion_model_noise rclcpp std_msgs geometry_msgs)
ament_target_dependencies(pf_based_motion rclcpp geometry_msgs std_msgs)
# Declare and install executables
install(TARGETS
motion_model
motion_model_noise
pf_based_motion
DESTINATION lib/${PROJECT_NAME}
)
# Pkg
ament_package()