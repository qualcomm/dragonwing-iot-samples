// The entire QRB ROS NPU API, in three calls.
//
//   1. QrbInferenceManager mgr(model_path, backend);   // load onto the NPU
//   2. mgr.inference_execute(input_bytes);             // run it
//   3. mgr.get_output_tensors();                       // get results back
//
// That is genuinely all of it. There is no session setup, no delegate
// registration, no graph builder, and no device management to write. This file
// runs a 515 MiB slice of a 3-billion-parameter vision-language-action model on
// a Hexagon NPU, and the interesting part is how little code it takes.
//
// Build (one line, no CMake needed). Note the include order: the workspace
// overlay MUST come before /opt/ros, or you compile against the older apt
// header and the link fails with an undefined reference to the constructor.
//   g++ -std=c++17 -O2 \
//     -I$PWD/ros2_ws/install/qrb_inference_manager/include \
//     workshop/examples/minimal_npu.cpp \
//     -L$PWD/ros2_ws/install/qrb_inference_manager/lib -lqrb_inference_manager \
//     -Wl,-rpath,$PWD/ros2_ws/install/qrb_inference_manager/lib \
//     -o /tmp/minimal_npu
//
// Run:
//   QNN_HTP_BURST=1 /tmp/minimal_npu \
//     artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075/vision_encoder.bin

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "qrb_inference_manager.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: minimal_npu <model.bin>\n");
    return 2;
  }
  const std::string model_path = argv[1];

  // Pi0.5's vision encoder takes one RGB image, 224x224, CHW, float32, in [-1, 1].
  constexpr int kElems = 3 * 224 * 224;
  std::vector<float> image(kElems);
  for (int i = 0; i < kElems; ++i) {
    image[i] = 0.5F;  // a flat grey frame is enough to prove the graph runs
  }

  // inference_execute() takes one flat byte buffer and slices it across the
  // graph's inputs in compile-time order. This graph has exactly one input, so
  // the buffer is just the image.
  std::vector<uint8_t> input(image.size() * sizeof(float));
  std::memcpy(input.data(), image.data(), input.size());

  // ---- 1. load ------------------------------------------------------------
  // "libQnnHtp.so" is what selects the Hexagon NPU. Swap it for libQnnCpu.so and
  // the same code runs on the CPU instead -- that one string is the whole
  // difference.
  std::printf("loading %s onto the NPU ...\n", model_path.c_str());
  qrb::inference_mgr::QrbInferenceManager mgr(model_path, "libQnnHtp.so");

  // ---- 2. run -------------------------------------------------------------
  if (!mgr.inference_execute(input)) {
    std::fprintf(stderr, "inference_execute failed\n");
    return 1;
  }

  // ---- 3. read the results ------------------------------------------------
  const auto outputs = mgr.get_output_tensors();
  std::printf("got %zu output tensor(s):\n", outputs.size());
  for (const auto & t : outputs) {
    std::printf("  %-12s bytes=%-9zu shape=[", t.output_tensor_name.c_str(),
        t.output_tensor_data.size());
    for (size_t i = 0; i < t.output_tensor_shape.size(); ++i) {
      std::printf("%s%u", i ? ", " : "", t.output_tensor_shape[i]);
    }
    std::printf("]\n");

    const auto * v = reinterpret_cast<const float *>(t.output_tensor_data.data());
    std::printf("               first 4 values: %.4f %.4f %.4f %.4f\n", v[0], v[1], v[2], v[3]);
  }

  std::printf("\nThat was a 3B-parameter model's vision encoder, on the NPU, in 3 API calls.\n");
  return 0;
}
