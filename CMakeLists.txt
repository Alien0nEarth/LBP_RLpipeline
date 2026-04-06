cmake_minimum_required(VERSION 3.8)
project(rl_quadrotor)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Reward.msg"
  "msg/Acc.msg"
  "msg/State.msg"
)

ament_package()
