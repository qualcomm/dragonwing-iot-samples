cmake_minimum_required(VERSION 3.16)
project(qrb_ros_vla)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgproc)
find_package(qrb_inference_manager REQUIRED)
find_package(qrb_ros_vla_msgs REQUIRED)

# --- core runner: no ROS dependency, so it is usable from the bench harness ---
add_library(pi05_runner SHARED src/pi05_runner.cpp)
target_include_directories(pi05_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pi05_runner qrb_inference_manager)

# --- standalone latency harness (bench/) ---
add_executable(pi05_bench src/pi05_bench.cpp)
target_link_libraries(pi05_bench pi05_runner)

# --- ROS 2 node ---
# PaliGemma tokenization for the live (state-aware) prompt path.
find_library(SENTENCEPIECE_LIB NAMES sentencepiece REQUIRED)
add_library(pi05_tokenizer SHARED src/pi05_tokenizer.cpp)
target_include_directories(pi05_tokenizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pi05_tokenizer ${SENTENCEPIECE_LIB})

add_library(vla_node_component SHARED src/vla_node.cpp)
target_include_directories(vla_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vla_node_component pi05_runner pi05_tokenizer ${OpenCV_LIBS})
ament_target_dependencies(vla_node_component
  rclcpp rclcpp_components std_msgs sensor_msgs cv_bridge qrb_ros_vla_msgs qrb_inference_manager)
rclcpp_components_register_node(vla_node_component
  PLUGIN "qrb_ros_vla::VlaNode"
  EXECUTABLE vla_node
)

install(TARGETS pi05_runner pi05_tokenizer vla_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS pi05_bench DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_libraries(pi05_runner)
ament_export_dependencies(rclcpp qrb_inference_manager qrb_ros_vla_msgs)
ament_package()
