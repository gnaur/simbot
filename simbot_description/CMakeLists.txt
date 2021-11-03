cmake_minimum_required(VERSION 3.5)
project(simbot_description)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(navigation2 REQUIRED)

#find_package(gazebo REQUIRED)
#find_package(gazebo_ros_pkgs REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

install(
  DIRECTORY src launch rviz meshes models worlds
  DESTINATION share/${PROJECT_NAME}
)

#ament_export_dependencies(gazebo_ros_pkgs)
ament_package()
