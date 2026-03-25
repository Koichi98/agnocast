// Internal header — kept in src/ so it is NOT installed or visible to downstream packages.
// Only get_backend.cpp includes this to instantiate the singleton.
#pragma once

#include "agnocast/gpu_transfer_backend.hpp"

namespace agnocast::cuda
{

// Placeholder backend using POSIX shared memory + cudaHostRegister for Jetson platforms
// where CPU and GPU share the same physical memory (unified memory architecture).
class UnifiedMemoryBackend : public GpuTransferBackend
{
public:
  GpuHandle export_handle(void * device_ptr, size_t size) override;
  void free_device_memory(void * device_ptr) override;
  void * import_handle(const GpuHandle & handle, size_t size) override;
  void release_handle(void * local_ptr) override;
};

}  // namespace agnocast::cuda
