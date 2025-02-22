cmake_minimum_required(VERSION 3.8)
project(omx_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kdl_parser REQUIRED)          # Add KDL parser
find_package(orocos_kdl REQUIRED)          # Add Orocos KDL

# Add executable
add_executable(omx_pick_place src/omx_pick_place.cpp)

# Link dependencies
ament_target_dependencies(omx_pick_place 
  rclcpp 
  trajectory_msgs 
  sensor_msgs 
  std_srvs 
  ament_index_cpp
  kdl_parser              # Link KDL parser
  orocos_kdl              # Link Orocos KDL
)

# Include directories
target_include_directories(omx_pick_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS}  # Include KDL headers
)

# Set C++ standard
target_compile_features(omx_pick_place PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install the executable
install(TARGETS omx_pick_place
  DESTINATION lib/${PROJECT_NAME})

# Linting and testing
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

# Finalize the package
ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(omx_pick_place)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(trajectory_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(ament_index_cpp REQUIRED)

# add_executable(omx_pick_place src/omx_pick_place.cpp)

# ament_target_dependencies(omx_pick_place 
#   rclcpp 
#   trajectory_msgs 
#   sensor_msgs 
#   std_srvs 
#   ament_index_cpp
# )

# target_include_directories(omx_pick_place PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(omx_pick_place PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS omx_pick_place
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()