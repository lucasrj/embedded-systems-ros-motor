cmake_minimum_required(VERSION 3.5)
project(embedded-systems-ros-motor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package( OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME} ${OpenCV_INCLUDE_DIRS})

add_library(nn_inference_driver STATIC src/nn_inference_driver/xnn_inference_linux.c src/nn_inference_driver/xnn_inference.c)

add_library(axi_dma STATIC src/axi_dma/axidma.cpp)

# # Specify the library output name and location
# set_target_properties(example_driver PROPERTIES OUTPUT_NAME example_driver)
# set_target_properties(example_driver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)


add_executable(servo_controll src/servo_controll.cpp)
ament_target_dependencies(servo_controll
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)

add_executable(ai_node src/ai_node.cpp)
target_link_libraries(ai_node nn_inference_driver axi_dma ${OpenCV_LIBS})
ament_target_dependencies(ai_node
  rclcpp
  cv_bridge
)

install(TARGETS servo_controll
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
