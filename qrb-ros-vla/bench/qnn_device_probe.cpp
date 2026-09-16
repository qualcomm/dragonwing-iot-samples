// Copyright (c) 2026 QRB ROS VLA contributors.
// SPDX-License-Identifier: BSD-3-Clause
//
// Ask the QNN HTP backend what NPU hardware it can actually see.
//
// Why this matters here: the IQ-9075 (QCS9075 / SA8775P) is documented as having
// dual Hexagon Tensor Processors, and Linux exposes /dev/fastrpc-cdsp and
// /dev/fastrpc-cdsp1. Pi0.5's four context binaries hold ~2.8 GiB of weights,
// which does NOT all fit in one CDSP's mapping budget -- so if QNN exposes two
// addressable HTP devices, the components could be split across them and stay
// resident, removing ~2 s/chunk of context paging. This probe answers whether
// that is even possible before anyone writes the plumbing.
//
// Build:
//   g++ -std=c++17 -O2 -I/usr/include/QNN bench/qnn_device_probe.cpp -ldl \
//       -o /tmp/qnn_device_probe

#include <dlfcn.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "QnnDevice.h"
#include "QnnInterface.h"
#include "QnnTypes.h"

namespace
{

using GetProvidersFn = Qnn_ErrorHandle_t (*)(const QnnInterface_t ***, uint32_t *);

const char * DeviceTypeName(uint32_t t)
{
  switch (t) {
    case 0:
      return "ON_CHIP";
    case 1:
      return "OFF_CHIP";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const char * lib = argc > 1 ? argv[1] : "libQnnHtp.so";

  void * handle = dlopen(lib, RTLD_NOW | RTLD_LOCAL);
  if (handle == nullptr) {
    std::fprintf(stderr, "dlopen(%s) failed: %s\n", lib, dlerror());
    return 1;
  }

  auto get_providers =
      reinterpret_cast<GetProvidersFn>(dlsym(handle, "QnnInterface_getProviders"));
  if (get_providers == nullptr) {
    std::fprintf(stderr, "QnnInterface_getProviders not found: %s\n", dlerror());
    return 1;
  }

  const QnnInterface_t ** providers = nullptr;
  uint32_t num_providers = 0;
  if (get_providers(&providers, &num_providers) != QNN_SUCCESS || num_providers == 0) {
    std::fprintf(stderr, "QnnInterface_getProviders returned no providers\n");
    return 1;
  }
  std::printf("backend            : %s\n", lib);
  std::printf("interface providers: %u\n", num_providers);

  const auto & api = providers[0]->QNN_INTERFACE_VER_NAME;
  std::printf("backend API version: %u.%u.%u\n", providers[0]->apiVersion.coreApiVersion.major,
      providers[0]->apiVersion.coreApiVersion.minor,
      providers[0]->apiVersion.coreApiVersion.patch);

  if (api.deviceGetPlatformInfo == nullptr) {
    std::printf("\ndeviceGetPlatformInfo is NOT implemented by this backend.\n"
                "=> QNN exposes no enumerable hardware device list; contexts cannot be\n"
                "   pinned to a specific NPU through this API.\n");
    return 0;
  }

  const QnnDevice_PlatformInfo_t * info = nullptr;
  const Qnn_ErrorHandle_t err = api.deviceGetPlatformInfo(nullptr, &info);
  if (err != QNN_SUCCESS || info == nullptr) {
    std::printf("\ndeviceGetPlatformInfo failed with 0x%llx\n",
        static_cast<unsigned long long>(err));
    return 0;
  }

  const auto & v1 = info->v1;
  std::printf("\nhardware devices   : %u\n", v1.numHwDevices);
  for (uint32_t i = 0; i < v1.numHwDevices; ++i) {
    const auto & dev = v1.hwDevices[i].v1;
    std::printf("  device[%u] id=%u type=%s numCores=%u\n", i, dev.deviceId,
        DeviceTypeName(dev.deviceType), dev.numCores);
    for (uint32_t c = 0; c < dev.numCores; ++c) {
      const auto & core = dev.cores[c].v1;
      std::printf("      core[%u] id=%u type=%u\n", c, core.coreId, core.coreType);
    }
  }

  std::printf("\nverdict: %s\n", v1.numHwDevices > 1
          ? "MULTIPLE HTP devices are addressable -- splitting Pi0.5 components across\n"
            "         them to avoid context paging is worth implementing."
          : "only ONE HTP device is addressable through QNN, so all contexts contend for\n"
            "         a single CDSP mapping budget. Context paging is unavoidable.");

  api.deviceFreePlatformInfo(nullptr, info);
  return 0;
}
