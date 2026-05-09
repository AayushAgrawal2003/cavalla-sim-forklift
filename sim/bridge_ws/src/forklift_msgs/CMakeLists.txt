cmake_minimum_required(VERSION 3.8)
project(forklift_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 1. FIND THE GENERATOR
# This package is responsible for turning .msg files into code
find_package(rosidl_default_generators REQUIRED)

# 2. DECLARE YOUR MESSAGES
# List every .msg file you created here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ForkliftDirectCommand.msg"
  "msg/ForkliftDriveFeedback.msg"
  "msg/AutomationCommand.msg"
  # If your messages use Header, uncomment the line below:
  # DEPENDENCIES std_msgs
)

# 3. EXPORT DEPENDENCIES
# This ensures other packages can find the generated code
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()