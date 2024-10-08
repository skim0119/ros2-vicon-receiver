cmake_minimum_required(VERSION 3.5)
project(vicon_receiver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Posture.msg"
  "msg/MarkersList.msg"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(
  vicon_client
  src/vicon_receiver/communicator.cpp
  src/vicon_receiver/publisher.cpp
)
target_link_libraries(vicon_client ViconDataStreamSDK_CPP ${Boost_LIBRARIES})
ament_target_dependencies(vicon_client "rclcpp")

add_executable(
  vicon_client_unlabeled
  src/vicon_unlabeled_marker_receiver/communicator.cpp
  src/vicon_unlabeled_marker_receiver/publisher.cpp
)
target_link_libraries(vicon_client_unlabeled ViconDataStreamSDK_CPP ${Boost_LIBRARIES})
ament_target_dependencies(vicon_client_unlabeled "rclcpp")

add_executable(
  vicon_client_mock
  src/vicon_receiver_mock/communicator.cpp
  src/vicon_receiver_mock/publisher.cpp 
  src/vicon_receiver_mock/data.cpp
)
target_link_libraries(vicon_client_mock ViconDataStreamSDK_CPP ${Boost_LIBRARIES})
ament_target_dependencies(vicon_client_mock "rclcpp")

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(vicon_client
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
  rosidl_target_interfaces(vicon_client_mock
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
  rosidl_target_interfaces(vicon_client_unlabeled
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
endforeach()

install(TARGETS vicon_client vicon_client_mock vicon_client_unlabeled DESTINATION lib/${PROJECT_NAME})

# Install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
