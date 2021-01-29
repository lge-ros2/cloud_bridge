cmake_minimum_required(VERSION 3.5)
project(cloud_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -O0)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(rcl REQUIRED)

set(DEPENDENCIES 
  "rclcpp"
  "geometry_msgs"
  "tf2_ros"
  "rcutils"
  "rcl"
)

set(LIBS
  tinyxml2
  zmq
  ${CMAKE_DL_LIBS}
)

set(CPPS
  src/cloud_bridge_base.cpp
  src/cloud_bridge_server.cpp
  src/cloud_bridge_client.cpp
  src/bridge_rcl_node.cpp
  src/zmq_transport.cpp
  src/message_types.cpp
)


include_directories(include)

set(BRIDGE_CLIENT_NAME "cloud_bridge_client")

add_executable(${BRIDGE_CLIENT_NAME}
  src/client_main.cpp
  ${CPPS}
)

ament_target_dependencies(${BRIDGE_CLIENT_NAME}
  ${DEPENDENCIES}
)

target_link_libraries(${BRIDGE_CLIENT_NAME}
  ${LIBS}
)

install(TARGETS ${BRIDGE_CLIENT_NAME} 
      DESTINATION lib/${CMAKE_PROJECT_NAME})


set(BRIDGE_SERVER_NAME "cloud_bridge_server")

add_executable(${BRIDGE_SERVER_NAME}
  src/server_main.cpp
  ${CPPS}
)

ament_target_dependencies(${BRIDGE_SERVER_NAME}
  ${DEPENDENCIES}
)

target_link_libraries(${BRIDGE_SERVER_NAME}
  ${LIBS}
)

install(TARGETS ${BRIDGE_SERVER_NAME} 
      DESTINATION lib/${CMAKE_PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
