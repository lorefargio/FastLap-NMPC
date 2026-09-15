cmake_minimum_required(VERSION 3.10)
project(etdv_mpc)

# ==============================================================================
# Compiler Flags & Optimization
# ==============================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -O3
    -march=native
    -mtune=native
    -flto
    -fomit-frame-pointer
  )
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# ROS 2 Dependencies
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pacsim REQUIRED)
find_package(Eigen3 REQUIRED)

# ==============================================================================
# acados Installation Lookup
# ==============================================================================
if(DEFINED ENV{ACADOS_SOURCE_DIR})
  set(ACADOS_DIR $ENV{ACADOS_SOURCE_DIR})
elseif(EXISTS "/opt/acados")
  set(ACADOS_DIR "/opt/acados")
elseif(EXISTS "$ENV{HOME}/acados")
  set(ACADOS_DIR "$ENV{HOME}/acados")
else()
  message(WARNING "ACADOS_SOURCE_DIR not found. Set ACADOS_SOURCE_DIR to your acados root.")
  set(ACADOS_DIR "/opt/acados")
endif()

message(STATUS "Using ACADOS_DIR: ${ACADOS_DIR}")

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${ACADOS_DIR}/include
  ${ACADOS_DIR}/include/acados
  ${ACADOS_DIR}/include/blasfeo/include
  ${ACADOS_DIR}/include/hpipm/include
  ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code
  ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/kinematic_frenet_model
  ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/kinematic_frenet_cost
  ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/kinematic_frenet_constraints
)

link_directories(
  ${ACADOS_DIR}/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code
)

# Explicit acados generated solver sources for kinematic_frenet
set(ACADOS_GENERATED_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_solver_kinematic_frenet.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_sim_solver_kinematic_frenet.c"
)
file(GLOB ACADOS_MODEL_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/kinematic_frenet_model/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/kinematic_frenet_cost/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/kinematic_frenet_constraints/*.c"
)
list(APPEND ACADOS_GENERATED_SOURCES ${ACADOS_MODEL_SOURCES})

if(ACADOS_GENERATED_SOURCES)
  # Suppress compiler warnings for auto-generated C code
  set_source_files_properties(${ACADOS_GENERATED_SOURCES} PROPERTIES COMPILE_FLAGS "-w")
else()
  message(WARNING "No acados C generated code found in ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code. Run 'python3 python_model/generate_c_code.py' before building.")
endif()

# ==============================================================================
# Controller Node Executable
# ==============================================================================
add_executable(mpc_pacsim_node
  src/mpc_pacsim_node.cpp
  src/acados_mpc_solver.cpp
  src/frenet_track.cpp
  src/utils.cpp
  ${ACADOS_GENERATED_SOURCES}
)

ament_target_dependencies(mpc_pacsim_node
  ament_index_cpp
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  pacsim
)

target_link_libraries(mpc_pacsim_node
  Eigen3::Eigen
  acados
  hpipm
  blasfeo
  m
  pthread
)

# ==============================================================================
# Installation
# ==============================================================================
install(TARGETS
  mpc_pacsim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  scripts
  skills
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
