cmake_minimum_required(VERSION 3.14)
project(agnocast_cuda LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(sensor_msgs REQUIRED)

# CUDA is optional: if not available, headers are still installed but no library is built.
# Downstream packages that use CUDA message types must check if the agnocast_cuda library
# target exists (e.g., via find_package + if(TARGET agnocast_cuda::agnocast_cuda)).
#
# We use find_package(CUDAToolkit) instead of check_language(CUDA) + enable_language(CUDA)
# because all sources are .cpp (no .cu files) — we only need to link against cudart, not
# invoke nvcc. This also avoids the common issue where check_language(CUDA) fails because
# nvcc is not on PATH even though the CUDA toolkit is installed.
find_package(CUDAToolkit QUIET)

if(CUDAToolkit_FOUND)
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

  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(agnocastlib sensor_msgs)

  message(STATUS "agnocast_cuda: CUDA found — building library and installing headers.")
else()
  message(STATUS "agnocast_cuda: CUDA not found — installing headers only (no library).")
endif()

# Always install headers so downstream packages can use the type definitions
# (e.g., cuda_message_tag.hpp is in agnocastlib, but types.hpp is here).
install(
  DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_package()
