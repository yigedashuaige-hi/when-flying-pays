cmake_minimum_required(VERSION 3.0.2)
project(rescue_worlds)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)

catkin_package()

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
