cmake_minimum_required(VERSION 3.8)
project(rm_seek)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
