#include "vmm_backend.hpp"

#include <stdexcept>

namespace agnocast::cuda
{

GpuHandle VmmBackend::export_handle(void * /*device_ptr*/, size_t /*size*/)
{
  throw std::runtime_error(
    "[agnocast_cuda] VmmBackend is not yet implemented. "
    "Requires cuMemExportToShareableHandle (CUDA Driver API).");
}

void VmmBackend::free_device_memory(void * /*device_ptr*/)
{
  throw std::runtime_error("[agnocast_cuda] VmmBackend is not yet implemented.");
}

void * VmmBackend::import_handle(const GpuHandle & /*handle*/, size_t /*size*/)
{
  throw std::runtime_error("[agnocast_cuda] VmmBackend is not yet implemented.");
}

void VmmBackend::release_handle(void * /*local_ptr*/)
{
  throw std::runtime_error("[agnocast_cuda] VmmBackend is not yet implemented.");
}

}  // namespace agnocast::cuda
