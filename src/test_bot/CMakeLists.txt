cmake_minimum_required(VERSION 3.8)
project(test_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependências adicionadas durante o código (inclui tipagem dos tópicos a serem publicados manualmente)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)


# chamada dos executáveis
#rodar o robô
add_executable(spin_robot src/spin_robot.cpp)
ament_target_dependencies(spin_robot rclcpp geometry_msgs) #adicionar todas as dependencias
#ler o scanner
add_executable(read_laser src/read_laser.cpp)
ament_target_dependencies(read_laser rclcpp sensor_msgs) 
#desviar de obstáculos
add_executable(avoid_obstacles src/avoid_obstacles.cpp)
ament_target_dependencies(avoid_obstacles rclcpp sensor_msgs geometry_msgs) 

install(TARGETS
read_laser
spin_robot
avoid_obstacles
DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY launch models worlds meshes config params rviz src
DESTINATION share/${PROJECT_NAME}/
)


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

ament_package()
