#include "cuda_ipc_backend.hpp"

#include <cuda_runtime.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{

void check_cuda_error(cudaError_t err, const char * operation)
{
  if (err != cudaSuccess) {
    std::fprintf(
      stderr, "[agnocast_cuda] FATAL: %s failed: %s\n", operation, cudaGetErrorString(err));
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
  check_cuda_error(cudaIpcGetMemHandle(&ipc_handle, device_ptr), "cudaIpcGetMemHandle");
  std::memcpy(h.opaque, &ipc_handle, sizeof(ipc_handle));
  return h;
}

void CudaIpcBackend::free_device_memory(void * device_ptr)
{
  check_cuda_error(cudaFree(device_ptr), "cudaFree");
}

void * CudaIpcBackend::import_handle(const GpuHandle & handle, size_t /*size*/)
{
  cudaIpcMemHandle_t ipc_handle;
  std::memcpy(&ipc_handle, handle.opaque, sizeof(ipc_handle));
  void * ptr = nullptr;
  check_cuda_error(
    cudaIpcOpenMemHandle(&ptr, ipc_handle, cudaIpcMemLazyEnablePeerAccess),
    "cudaIpcOpenMemHandle");
  return ptr;
}

void CudaIpcBackend::release_handle(void * local_ptr)
{
  check_cuda_error(cudaIpcCloseMemHandle(local_ptr), "cudaIpcCloseMemHandle");
}

}  // namespace agnocast::cuda
