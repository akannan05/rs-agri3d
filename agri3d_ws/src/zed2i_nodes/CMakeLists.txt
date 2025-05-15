cmake_minimum_required(VERSION 3.8)
project(zed2i_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# include zed-open-capture dep

set(ZED_OC_PATH "$ENV{HOME}/Desktop/zed-open-capture") # set this to the directory where you have installed zed-oc

find_library(ZED_OC_LIB
    NAMES zed_open_capture
    PATHS ${ZED_OC_PATH}/build
)

add_executable(zed_imu_node src/zed_imu_publisher.cpp)
add_executable(zed_img_node src/zed_img_publisher.cpp)

target_include_directories(zed_imu_node PRIVATE
    ${ZED_OC_PATH}/include
)

target_link_libraries(zed_imu_node
    ${ZED_OC_LIB}
)

target_include_directories(zed_img_node PRIVATE
    ${ZED_OC_PATH}/include
)

target_link_libraries(zed_img_node
    ${ZED_OC_LIB}
)

ament_target_dependencies(zed_imu_node
    rclcpp
    std_msgs
    sensor_msgs
)

ament_target_dependencies(zed_img_node
    rclcpp
    image_transport
    sensor_msgs
    cv_bridge
)

install(TARGETS
    zed_imu_node
    zed_img_node
    DESTINATION lib/${PROJECT_NAME}
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
