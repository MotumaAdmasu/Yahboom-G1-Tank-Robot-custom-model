cmake_minimum_required(VERSION 3.8)
project(yahboom_g1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/simple_tank_movement.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/tank_controller_working.py
  scripts/test_movement.py
  scripts/mobile_tank_controller.py
  scripts/real_movement_controller.py
  scripts/correct_wheel_controller.py
  scripts/proper_wheel_rolling.py
  DESTINATION lib/${PROJECT_NAME}
)
# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
# Install Python scripts
install(PROGRAMS
  scripts/tank_controller_working.py
  scripts/test_movement.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/tank_controller_working.py
  scripts/test_movement.py
  scripts/mobile_tank_controller.py
  scripts/real_movement_controller.py
  scripts/correct_wheel_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/tank_controller_working.py
  scripts/test_movement.py
  scripts/mobile_tank_controller.py
  scripts/real_movement_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
# Install RViz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/tank_controller_working.py
  scripts/test_movement.py
  scripts/mobile_tank_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
# Install config files (ADD THIS LINE)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
