cmake_minimum_required(VERSION 3.16)
project(exam_proctor_gz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(exam_proctor_student_animator SHARED src/student_animator.cpp)
target_link_libraries(exam_proctor_student_animator
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2-register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS exam_proctor_student_animator LIBRARY DESTINATION lib)

# Let Gazebo find the plugin once the workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
