cmake_minimum_required(VERSION 3.8)
project(meamr_drive_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)       #+
find_package(geometry_msgs REQUIRED)  #+
find_package(tf2 REQUIRED)            #+    
find_package(Boost REQUIRED COMPONENTS thread)  #+




include_directories(include ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../serial/include)
# add_subdirectory(../serial ${CMAKE_CURRENT_BINARY_DIR}/serial_build)  # <-- 加這行！

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

add_executable(meamr_base_node
  src/meamr_base_node.cpp
  src/meamr_base.cpp         # 這行很重要
  src/dd_kinematic_model.cpp  
  # src/serial_motors.cpp   # ✅ 要加這個！
  # src/serial_port.cpp    # 如果有的話也加
)
# target_link_libraries(meamr_base_node
#   serial
# )

ament_target_dependencies(meamr_base_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
)

install(TARGETS
  meamr_base_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
