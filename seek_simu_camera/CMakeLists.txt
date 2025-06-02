cmake_minimum_required(VERSION 3.8)
project(seek_simu_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 找到依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

# 添加库
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  DIRECTORY src
)

# 链接依赖项
target_link_libraries(${PROJECT_NAME}_lib ${OpenCV_LIBS} ${EIGEN3_LIBS})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN fyt::seek::SeekSimuCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
