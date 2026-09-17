cmake_minimum_required(VERSION 3.8)
project(mir100_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params rviz2 maps scripts
  DESTINATION share/${PROJECT_NAME}
)

# Also install for smart_farm_navigation so both package names are valid in ros2 launch
install(DIRECTORY launch params rviz2 maps scripts
  DESTINATION share/smart_farm_navigation
)

ament_package()
