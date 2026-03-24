// Internal header — kept in src/ so it is NOT installed or visible to downstream packages.
// Only get_backend.cpp includes this to instantiate the singleton.
#pragma once

#include "agnocast/gpu_transfer_backend.hpp"

namespace agnocast::cuda
{

// Placeholder backend using CUDA Virtual Memory Management (VMM) API.
// Uses cuMemExportToShareableHandle / cuMemImportFromShareableHandle.
// Supported on Jetson Orin (newer CUDA) and discrete GPUs with CUDA 10.2+.
class VmmBackend : public GpuTransferBackend
{
public:
  GpuHandle export_handle(void * device_ptr, size_t size) override;
  void free_device_memory(void * device_ptr) override;
  void * import_handle(const GpuHandle & handle, size_t size) override;
  void release_handle(void * local_ptr) override;
};

}  // namespace agnocast::cuda
