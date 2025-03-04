cmake_minimum_required(VERSION 3.8)
project(imu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

# 手动设置serial库路径
# get_filename_component(serial ${serial_DIR} DIRECTORY)
set(SERIAL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../serial/include")
set(SERIAL_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../serial/build/libserial.a")

add_executable(imu_driver_exe src/imu.cpp)
target_include_directories(imu_driver_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SERIAL_INCLUDE_DIR}) # 添加serial库头文件路径
target_link_libraries(imu_driver_exe ${SERIAL_LIBRARY}) # 链接serial库文件
target_compile_features(imu_driver_exe PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  imu_driver_exe
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "tf2"
)

install(TARGETS imu_driver_exe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

