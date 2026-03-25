#include "agnocast/gpu_transfer_backend.hpp"
#include "cuda_ipc_backend.hpp"
#include "nvscibuf_backend.hpp"
#include "unified_memory_backend.hpp"
#include "vmm_backend.hpp"

#include <cuda_runtime.h>

#include <cstdio>
#include <cstdlib>
#include <memory>

namespace agnocast::cuda
{

namespace
{

std::unique_ptr<GpuTransferBackend> select_backend()
{
  int device = 0;
  cudaError_t err = cudaGetDevice(&device);
  if (err != cudaSuccess) {
    std::fprintf(
      stderr, "[agnocast_cuda] FATAL: cudaGetDevice failed: %s\n", cudaGetErrorString(err));
    std::abort();
  }

  int is_integrated = 0;
  err = cudaDeviceGetAttribute(&is_integrated, cudaDevAttrIntegrated, device);
  if (err != cudaSuccess) {
    std::fprintf(
      stderr, "[agnocast_cuda] FATAL: cudaDeviceGetAttribute failed: %s\n",
      cudaGetErrorString(err));
    std::abort();
  }

  if (!is_integrated) {
    // Discrete GPU (GeForce, Quadro, Tesla, A/H series) — CUDA IPC is supported.
    std::fprintf(stderr, "[agnocast_cuda] Discrete GPU detected, using CudaIpcBackend.\n");
    return std::make_unique<CudaIpcBackend>();
  }

  // Integrated GPU (Jetson Xavier/Orin/Thor, DRIVE).
  // TODO(agnocast): Implement and select the appropriate backend.
  //   - Jetson Thor (CUDA 13.0+): CudaIpcBackend may work via OpenRM.
  //   - Jetson Xavier/Orin: NvSciBufBackend or UnifiedMemoryBackend.
  //   - DRIVE: NvSciBufBackend.
  std::fprintf(
    stderr,
    "[agnocast_cuda] FATAL: Integrated GPU detected (Jetson/DRIVE). "
    "No backend is implemented yet for this platform.\n");
  std::abort();
}

}  // namespace

GpuTransferBackend & get_backend()
{
  static auto instance = select_backend();
  return *instance;
}

}  // namespace agnocast::cuda
