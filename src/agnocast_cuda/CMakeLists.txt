cmake_minimum_required(VERSION 3.14)
project(agnocast_cuda LANGUAGES CXX CUDA)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_library(agnocast_cuda SHARED
  src/cuda_ipc_backend.cpp
  src/vmm_backend.cpp
  src/nvscibuf_backend.cpp
  src/unified_memory_backend.cpp
  src/get_backend.cpp)

target_include_directories(agnocast_cuda PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(agnocast_cuda agnocastlib sensor_msgs)
target_link_libraries(agnocast_cuda CUDA::cudart)

install(TARGETS agnocast_cuda
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(
  DIRECTORY include/
  DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(agnocastlib sensor_msgs)
ament_package()
