#include "unified_memory_backend.hpp"

#include <stdexcept>

namespace agnocast::cuda
{

GpuHandle UnifiedMemoryBackend::export_handle(void * /*device_ptr*/, size_t /*size*/)
{
  throw std::runtime_error(
    "[agnocast_cuda] UnifiedMemoryBackend is not yet implemented. "
    "Requires POSIX shm + cudaHostRegister (Jetson unified memory).");
}

void UnifiedMemoryBackend::free_device_memory(void * /*device_ptr*/)
{
  throw std::runtime_error("[agnocast_cuda] UnifiedMemoryBackend is not yet implemented.");
}

void * UnifiedMemoryBackend::import_handle(const GpuHandle & /*handle*/, size_t /*size*/)
{
  throw std::runtime_error("[agnocast_cuda] UnifiedMemoryBackend is not yet implemented.");
}

void UnifiedMemoryBackend::release_handle(void * /*local_ptr*/)
{
  throw std::runtime_error("[agnocast_cuda] UnifiedMemoryBackend is not yet implemented.");
}

}  // namespace agnocast::cuda
