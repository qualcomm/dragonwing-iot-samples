// Copyright (c) 2026 QRB ROS VLA contributors.
// SPDX-License-Identifier: BSD-3-Clause
//
// Pi05Runner: runs the four-component Qualcomm AI Hub Pi0.5 export on the
// Hexagon HTP NPU through qrb_inference_manager (QRB ROS's QNN wrapper).
//
// One Infer() call is one action chunk and costs, on the NPU:
//   3x vision_encoder  +  1x token_emb  +  1x backbone  +  Nx action_expert
// where N = denoise_steps (upstream default 10, from lerobot/pi05_libero
// "num_inference_steps": 10).
//
// Two hardware realities shape this class:
//
//  1. CDSP SMMU MAPPING CEILING. The four context binaries hold ~3.0 GB of
//     weights (token_emb 1055 MB, backbone 980 MB, vision_encoder 540 MB,
//     action_expert 439 MB). Mapping all four into the CDSP at once fails on an
//     IQ-9075 with "Failed to map buffer of size ..." / err 1002, even as root
//     and with 34 GB of free host RAM -- the limit is DSP-side address space,
//     not memory. Each binary loads fine alone. So components are split into a
//     resident set and a rotating set: rotating components are created before
//     use and destroyed after. Default residency keeps the two hot components
//     (vision_encoder, 3 calls/chunk; action_expert, N calls/chunk) mapped and
//     rotates the two single-call ones.
//
//  2. Robot state is NOT a tensor input anywhere in this graph. Pi0.5
//     discretizes proprioceptive state into text and folds it into the task
//     string before tokenization, so it arrives inside lang_tokens.
//
// See docs/DESIGN.md for the measured numbers behind both points.

#ifndef QRB_ROS_VLA__PI05_RUNNER_HPP_
#define QRB_ROS_VLA__PI05_RUNNER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace qrb::inference_mgr
{
class QrbInferenceManager;
}

namespace qrb_ros_vla
{

// Shapes baked into the exported context binaries. Cross-checked against the
// generated graph-order header at compile time, so a re-export with different
// dimensions fails loudly instead of producing wrong actions.
constexpr int kNumCameras = 3;
constexpr int kImageH = 224;
constexpr int kImageW = 224;
constexpr int kImageElems = 3 * kImageH * kImageW;  // 150528, CHW
constexpr int kMaxTokenLength = 200;
constexpr int kActionSteps = 50;   // action chunk length
constexpr int kActionDim = 32;     // max_action_dim; real DoF is a prefix of this
constexpr int kNumLayers = 18;
constexpr int kDefaultDenoiseSteps = 10;

enum class Component
{
  kVisionEncoder = 0,
  kTokenEmb,
  kBackbone,
  kActionExpert,
  kCount
};

// "vision_encoder", "token_emb", "backbone", "action_expert"
const char * ComponentName(Component c);
// Returns Component::kCount if the name is not recognised.
Component ComponentFromName(const std::string & name);

struct Pi05Timings
{
  double vision_ms = 0.0;         // all kNumCameras encoder passes
  double token_emb_ms = 0.0;
  double backbone_ms = 0.0;
  double action_expert_ms = 0.0;  // all denoise steps
  double pack_ms = 0.0;           // host-side buffer assembly
  double context_ms = 0.0;        // creating/destroying rotating contexts
  double total_ms = 0.0;
  int denoise_steps = 0;
};

struct Pi05Observation
{
  // Up to kNumCameras images, each kImageElems floats, RGB, CHW, already
  // resized to 224x224 and normalized to [-1, 1]. Missing slots are
  // zero-filled, which is what the upstream export does for `empty_cameras`.
  std::vector<std::vector<float>> images;
  std::vector<int32_t> lang_tokens;  // exactly kMaxTokenLength
  std::vector<float> lang_mask;      // exactly kMaxTokenLength, 1.0 = real token
};

struct Pi05ActionChunk
{
  std::vector<float> actions;  // kActionSteps * kActionDim, row-major
  Pi05Timings timings;
};

class Pi05Runner
{
public:
  struct Options
  {
    std::string bundle_dir;                 // dir holding the four .bin files
    std::string backend = "libQnnHtp.so";   // dlopen'd by qrb_inference_manager
    int denoise_steps = kDefaultDenoiseSteps;
    uint64_t seed = 0;                      // 0 => nondeterministic noise
    // Components kept mapped for the process lifetime. Everything not listed is
    // created before use and destroyed after. An empty list means "rotate
    // everything", which minimises peak DSP mapping at the cost of four context
    // creations per chunk.
    std::vector<std::string> resident = {
        "vision_encoder", "token_emb", "backbone", "action_expert"};
    // HTP hardware device per component, indexed by Component. QCS9075 exposes
    // two (bench/qnn_device_probe.cpp), each with its own CDSP mapping budget,
    // so splitting the four contexts across both lets all of them stay resident
    // and removes context paging entirely (~2-3 s/chunk saved).
    //
    // The two NSPs are NOT equally fast: measured on IQ-9075, every component
    // runs ~25-30% slower on device 1 than device 0 (backbone 513 vs 656 ms,
    // action_expert 389 vs 483 ms, vision 140 vs 196 ms). So the default puts
    // the expensive pair on device 0 and the cheap pair on device 1:
    //   device 0: backbone (934 MiB) + action_expert (419 MiB) = 1353 MiB
    //   device 1: vision_encoder (515 MiB) + token_emb (1007 MiB) = 1522 MiB
    // Measured 1131 ms/chunk, vs 1322 ms for the naive size-balanced split.
    std::vector<uint32_t> device_ids = {1, 1, 0, 0};
    // When set, every stage's flat input buffer and each named output tensor is
    // written here as raw little-endian floats/int32s on the FIRST Infer() call
    // only. Feeds bench/verify-against-qnn-net-run.sh, which replays the same
    // inputs through the stock qnn-net-run tool and diffs the outputs. This is
    // how we prove the hand-packed graph ordering is right rather than merely
    // non-degenerate.
    std::string dump_dir;
  };

  explicit Pi05Runner(Options options);
  ~Pi05Runner();

  Pi05Runner(const Pi05Runner &) = delete;
  Pi05Runner & operator=(const Pi05Runner &) = delete;

  // Verifies all four binaries exist and instantiates the resident set. Returns
  // false and sets *error on failure.
  bool Load(std::string * error);

  bool Infer(const Pi05Observation & obs, Pi05ActionChunk * out, std::string * error);

  const Options & options() const { return options_; }
  bool IsResident(Component c) const;
  // Bytes of weights this component maps into the CDSP, from the on-disk size.
  uint64_t WeightBytes(Component c) const;
  uint32_t DeviceId(Component c) const;

private:
  using Mgr = qrb::inference_mgr::QrbInferenceManager;

  // Creates the context if absent. No-op when already held.
  bool Acquire(Component c, double * context_ms, std::string * error);
  // Destroys the context unless the component is resident.
  void Release(Component c, double * context_ms);

  bool RunComponent(Component c,
      const std::vector<uint8_t> & inputs,
      std::unordered_map<std::string, std::vector<uint8_t>> * outputs,
      double * elapsed_ms,
      std::string * error);

  Options options_;
  std::array<std::unique_ptr<Mgr>, static_cast<size_t>(Component::kCount)> mgrs_;
  std::array<bool, static_cast<size_t>(Component::kCount)> resident_{};
  std::array<uint64_t, static_cast<size_t>(Component::kCount)> weight_bytes_{};
  std::array<int, static_cast<size_t>(Component::kCount)> call_counts_{};
  bool loaded_ = false;
  bool dumped_ = false;

  // Reused across calls so a steady-state Infer() does no large allocation.
  std::vector<uint8_t> vision_in_;
  std::vector<uint8_t> token_in_;
  std::vector<uint8_t> backbone_in_;
  std::vector<uint8_t> expert_in_;
  uint64_t rng_state_ = 0;
};

}  // namespace qrb_ros_vla

#endif  // QRB_ROS_VLA__PI05_RUNNER_HPP_
