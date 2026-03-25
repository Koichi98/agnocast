#pragma once

#include <cstddef>
#include <cstdint>

namespace agnocast
{

// Opaque handle large enough for cudaIpcMemHandle_t (64 bytes).
struct GpuHandle
{
  uint8_t opaque[64];
};

// GPU sharing metadata stored in shared memory alongside the message.
// Allocated by the publish path (while heaphook is active) so it lands in the publisher's
// shared memory region and is readable by subscribers.
struct GpuMetadata
{
  GpuHandle handle;          // backend-specific shareable handle
  size_t gpu_data_size;      // size of the GPU allocation in bytes
  void * publisher_gpu_ptr;  // original device pointer for publisher-side free on reclaim
};

}  // namespace agnocast
