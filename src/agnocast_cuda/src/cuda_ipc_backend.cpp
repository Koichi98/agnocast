#include "cuda_ipc_backend.hpp"

#include "cudart_loader.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{

void check_cuda_error(agnocast::cuda::cudaError_t err, const char * operation)
{
  if (err != agnocast::cuda::cudaSuccess) {
    std::fprintf(
      stderr, "[agnocast_cuda] FATAL: %s failed: %s\n", operation,
      agnocast::cuda::CudartLoader::instance().cudaGetErrorString(err));
    std::abort();
  }
}

}  // namespace

namespace agnocast::cuda
{

GpuHandle CudaIpcBackend::export_handle(void * device_ptr, size_t /*size*/)
{
  GpuHandle h{};
  static_assert(sizeof(cudaIpcMemHandle_t) <= sizeof(h.opaque));
  cudaIpcMemHandle_t ipc_handle;
  check_cuda_error(
    CudartLoader::instance().cudaIpcGetMemHandle(&ipc_handle, device_ptr), "cudaIpcGetMemHandle");
  std::memcpy(h.opaque, &ipc_handle, sizeof(ipc_handle));
  return h;
}

void CudaIpcBackend::free_device_memory(void * device_ptr)
{
  check_cuda_error(CudartLoader::instance().cudaFree(device_ptr), "cudaFree");
}

void * CudaIpcBackend::import_handle(const GpuHandle & handle, size_t /*size*/)
{
  cudaIpcMemHandle_t ipc_handle;
  std::memcpy(&ipc_handle, handle.opaque, sizeof(ipc_handle));
  void * ptr = nullptr;
  check_cuda_error(
    CudartLoader::instance().cudaIpcOpenMemHandle(&ptr, ipc_handle, cudaIpcMemLazyEnablePeerAccess),
    "cudaIpcOpenMemHandle");
  return ptr;
}

void CudaIpcBackend::release_handle(void * local_ptr)
{
  check_cuda_error(
    CudartLoader::instance().cudaIpcCloseMemHandle(local_ptr), "cudaIpcCloseMemHandle");
}

}  // namespace agnocast::cuda
