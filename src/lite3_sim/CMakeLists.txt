cmake_minimum_required(VERSION 3.8)
project(lite3_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch worlds config
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/make_sim_urdf.py scripts/depth_to_mm.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
