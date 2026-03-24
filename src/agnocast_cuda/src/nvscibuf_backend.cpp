#include "nvscibuf_backend.hpp"

#include <stdexcept>

namespace agnocast::cuda
{

GpuHandle NvSciBufBackend::export_handle(void * /*device_ptr*/, size_t /*size*/)
{
  throw std::runtime_error(
    "[agnocast_cuda] NvSciBufBackend is not yet implemented. "
    "Requires NvSciBuf (Jetson Xavier/Orin, NVIDIA DRIVE).");
}

void NvSciBufBackend::free_device_memory(void * /*device_ptr*/)
{
  throw std::runtime_error("[agnocast_cuda] NvSciBufBackend is not yet implemented.");
}

void * NvSciBufBackend::import_handle(const GpuHandle & /*handle*/, size_t /*size*/)
{
  throw std::runtime_error("[agnocast_cuda] NvSciBufBackend is not yet implemented.");
}

void NvSciBufBackend::release_handle(void * /*local_ptr*/)
{
  throw std::runtime_error("[agnocast_cuda] NvSciBufBackend is not yet implemented.");
}

}  // namespace agnocast::cuda
