#pragma once

#include "agnocast/gpu_metadata.hpp"

#include <cstddef>

namespace agnocast::cuda
{

// Abstract interface for GPU memory sharing backends.
// Concrete implementations (CudaIpcBackend, etc.) live in the agnocast_cuda package.
class GpuTransferBackend
{
public:
  virtual ~GpuTransferBackend() = default;

  // Publisher side: create a shareable handle from a device pointer.
  virtual GpuHandle export_handle(void * device_ptr, size_t size) = 0;

  // Publisher side: free GPU buffer on reclaim.
  virtual void free_device_memory(void * device_ptr) = 0;

  // Subscriber side: map GPU buffer into this process.
  virtual void * import_handle(const GpuHandle & handle, size_t size) = 0;

  // Subscriber side: unmap GPU buffer from this process.
  virtual void release_handle(void * local_ptr) = 0;
};

// Returns the singleton backend instance. Defined in the agnocast_cuda package and resolved
// at link time. Only called from if-constexpr branches guarded by is_cuda_message_v<T>,
// so the symbol is never referenced unless a CUDA message type is actually used.
GpuTransferBackend & get_backend();

}  // namespace agnocast::cuda
