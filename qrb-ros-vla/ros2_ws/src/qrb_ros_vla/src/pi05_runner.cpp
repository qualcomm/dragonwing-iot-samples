// Copyright (c) 2026 QRB ROS VLA contributors.
// SPDX-License-Identifier: BSD-3-Clause

#include "qrb_ros_vla/pi05_runner.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>

#include "qrb_inference_manager.hpp"
#include "qrb_ros_vla/pi05_graph_order.hpp"

namespace qrb_ros_vla
{
namespace
{

using qrb::inference_mgr::QrbInferenceManager;

constexpr size_t kComponentCount = static_cast<size_t>(Component::kCount);

const char * const kComponentNames[kComponentCount] = {
  "vision_encoder",
  "token_emb",
  "backbone",
  "action_expert",
};

double NowMs()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch()).count();
}

template <size_t N>
size_t TotalBytes(const pi05::TensorSpec (&specs)[N])
{
  size_t total = 0;
  for (const auto & s : specs) {
    total += s.byte_count;
  }
  return total;
}

// Byte offset of each tensor inside the flat buffer that inference_execute()
// consumes. Order is the compiled graph order -- see pi05_graph_order.hpp.
template <size_t N>
std::unordered_map<std::string, size_t> OffsetMap(const pi05::TensorSpec (&specs)[N])
{
  std::unordered_map<std::string, size_t> offsets;
  size_t offset = 0;
  for (const auto & s : specs) {
    offsets.emplace(s.name, offset);
    offset += s.byte_count;
  }
  return offsets;
}

// backbone emits k_cache_lN / v_cache_lN; action_expert wants
// key_cache_lN / value_cache_lN, and in lexicographic layer order. token_emb
// emits suffix_sin/suffix_cos; action_expert calls them rope_emb_sin/cos.
std::string ExpertInputSource(const std::string & input_name)
{
  static constexpr char kKey[] = "key_cache_l";
  static constexpr char kValue[] = "value_cache_l";
  if (input_name.rfind(kKey, 0) == 0) {
    return "k_cache_l" + input_name.substr(sizeof(kKey) - 1);
  }
  if (input_name.rfind(kValue, 0) == 0) {
    return "v_cache_l" + input_name.substr(sizeof(kValue) - 1);
  }
  if (input_name == "rope_emb_cos") {
    return "suffix_cos";
  }
  if (input_name == "rope_emb_sin") {
    return "suffix_sin";
  }
  if (input_name == "full_att_4d") {
    return "full_att_4d";
  }
  return {};  // x_t and time_step are produced locally by the denoise loop.
}

// token_emb emits prefix_*; backbone names the same tensors differently, and
// swaps cos/sin relative to the producer.
std::string BackboneInputSource(const std::string & input_name)
{
  if (input_name == "prefix_att_2d_masks") {
    return "prefix_att_2d";
  }
  if (input_name == "hidden_state") {
    return "prefix_emb";
  }
  if (input_name == "rope_emb_cos") {
    return "prefix_cos";
  }
  if (input_name == "rope_emb_sin") {
    return "prefix_sin";
  }
  return {};
}

bool CopyNamed(const std::unordered_map<std::string, std::vector<uint8_t>> & src,
    const std::string & name,
    uint8_t * dst,
    size_t expected_bytes,
    std::string * error)
{
  auto it = src.find(name);
  if (it == src.end()) {
    *error = "producer did not emit tensor '" + name + "'";
    return false;
  }
  if (it->second.size() != expected_bytes) {
    std::ostringstream os;
    os << "tensor '" << name << "' is " << it->second.size() << " bytes, graph wants "
       << expected_bytes;
    *error = os.str();
    return false;
  }
  std::memcpy(dst, it->second.data(), expected_bytes);
  return true;
}

size_t ExpectedOutputCount(Component c)
{
  switch (c) {
    case Component::kVisionEncoder:
      return std::size(pi05::kVisionEncoderOutputs);
    case Component::kTokenEmb:
      return std::size(pi05::kTokenEmbOutputs);
    case Component::kBackbone:
      return std::size(pi05::kBackboneOutputs);
    case Component::kActionExpert:
      return std::size(pi05::kActionExpertOutputs);
    default:
      return 0;
  }
}

void DumpRaw(const std::string & path, const void * data, size_t bytes)
{
  std::ofstream out(path, std::ios::binary);
  if (out) {
    out.write(static_cast<const char *>(data), static_cast<std::streamsize>(bytes));
  }
}

}  // namespace

const char * ComponentName(Component c)
{
  const size_t i = static_cast<size_t>(c);
  return i < kComponentCount ? kComponentNames[i] : "<invalid>";
}

Component ComponentFromName(const std::string & name)
{
  for (size_t i = 0; i < kComponentCount; ++i) {
    if (name == kComponentNames[i]) {
      return static_cast<Component>(i);
    }
  }
  return Component::kCount;
}

Pi05Runner::Pi05Runner(Options options) : options_(std::move(options))
{
  if (options_.denoise_steps < 1) {
    throw std::invalid_argument("denoise_steps must be >= 1");
  }
  // Fail loudly now if the committed graph-order header disagrees with the
  // shapes this code assumes. Cheaper than debugging silently wrong actions.
  static_assert(pi05::kVisionEncoderInputs[0].elem_count == kImageElems,
      "vision_encoder image tensor does not match kImageElems");
  static_assert(pi05::kActionExpertInputs[0].elem_count == kActionSteps * kActionDim,
      "action_expert x_t does not match kActionSteps * kActionDim");
  static_assert(pi05::kActionExpertOutputs[0].elem_count == kActionSteps * kActionDim,
      "action_expert action_emb does not match kActionSteps * kActionDim");
  static_assert(std::size(pi05::kBackboneOutputs) == 2 * kNumLayers,
      "backbone should emit one K and one V cache per layer");
  static_assert(std::size(pi05::kTokenEmbInputs) == kNumCameras + 2,
      "token_emb should take one embedding per camera plus tokens and mask");

  resident_.fill(false);
  for (const auto & name : options_.resident) {
    const Component c = ComponentFromName(name);
    if (c == Component::kCount) {
      throw std::invalid_argument("unknown component in 'resident': " + name);
    }
    resident_[static_cast<size_t>(c)] = true;
  }
}

Pi05Runner::~Pi05Runner() = default;

bool Pi05Runner::IsResident(Component c) const
{
  return resident_[static_cast<size_t>(c)];
}

uint64_t Pi05Runner::WeightBytes(Component c) const
{
  return weight_bytes_[static_cast<size_t>(c)];
}

uint32_t Pi05Runner::DeviceId(Component c) const
{
  const size_t i = static_cast<size_t>(c);
  return i < options_.device_ids.size() ? options_.device_ids[i] : 0U;
}

bool Pi05Runner::Load(std::string * error)
{
  namespace fs = std::filesystem;
  const fs::path dir(options_.bundle_dir);

  for (size_t i = 0; i < kComponentCount; ++i) {
    const fs::path path = dir / (std::string(kComponentNames[i]) + ".bin");
    std::error_code ec;
    if (!fs::exists(path, ec)) {
      *error = "missing context binary: " + path.string();
      return false;
    }
    weight_bytes_[i] = static_cast<uint64_t>(fs::file_size(path, ec));
  }

  // Each HTP device has its own CDSP mapping budget, so account per device.
  //
  // Measured on IQ-9075 with everything on device 0: a 1523 MiB peak is clean,
  // a 1941 MiB peak logs "Failed to map buffer" / err 1002 (and 2875 MiB, all
  // four contexts, fails outright). Note the failures are not a pure volume
  // limit -- they also depend on fragmentation from repeated map/unmap -- so the
  // ceiling below is deliberately conservative rather than the largest value
  // ever seen to work.
  constexpr uint64_t kSoftCeilingBytesPerDevice = 1600ULL * 1024 * 1024;
  std::unordered_map<uint32_t, uint64_t> resident_by_device;
  for (size_t i = 0; i < kComponentCount; ++i) {
    if (resident_[i]) {
      resident_by_device[DeviceId(static_cast<Component>(i))] += weight_bytes_[i];
    }
  }
  for (size_t i = 0; i < kComponentCount; ++i) {
    const uint32_t dev = DeviceId(static_cast<Component>(i));
    const uint64_t base = resident_by_device.count(dev) ? resident_by_device[dev] : 0;
    const uint64_t peak = resident_[i] ? base : base + weight_bytes_[i];
    if (peak > kSoftCeilingBytesPerDevice) {
      std::ostringstream os;
      os << "HTP device " << dev << " would map up to " << (peak >> 20)
         << " MiB of weights at once, above the ~" << (kSoftCeilingBytesPerDevice >> 20)
         << " MiB ceiling measured on IQ-9075; move a component to the other device via"
            " 'device_ids' or drop it from 'resident'";
      *error = os.str();
      return false;
    }
  }

  double context_ms = 0.0;
  for (size_t i = 0; i < kComponentCount; ++i) {
    if (resident_[i] && !Acquire(static_cast<Component>(i), &context_ms, error)) {
      return false;
    }
  }

  vision_in_.resize(TotalBytes(pi05::kVisionEncoderInputs));
  token_in_.resize(TotalBytes(pi05::kTokenEmbInputs));
  backbone_in_.resize(TotalBytes(pi05::kBackboneInputs));
  expert_in_.resize(TotalBytes(pi05::kActionExpertInputs));

  if (!options_.dump_dir.empty()) {
    std::error_code ec;
    fs::create_directories(options_.dump_dir, ec);
    if (ec) {
      *error = "cannot create dump_dir " + options_.dump_dir + ": " + ec.message();
      return false;
    }
  }

  rng_state_ = options_.seed != 0 ? options_.seed : std::random_device{}();
  loaded_ = true;
  return true;
}

bool Pi05Runner::Acquire(Component c, double * context_ms, std::string * error)
{
  const size_t i = static_cast<size_t>(c);
  if (mgrs_[i] != nullptr) {
    return true;
  }
  const std::filesystem::path path =
      std::filesystem::path(options_.bundle_dir) / (std::string(kComponentNames[i]) + ".bin");
  const double start = NowMs();
  try {
    mgrs_[i] = std::make_unique<Mgr>(path.string(), options_.backend, DeviceId(static_cast<Component>(i)));
  } catch (const std::exception & e) {
    *error = std::string("failed to create context for ") + kComponentNames[i] + ": " + e.what();
    return false;
  }
  *context_ms += NowMs() - start;
  if (mgrs_[i] == nullptr) {
    *error = std::string("failed to create context for ") + kComponentNames[i];
    return false;
  }
  return true;
}

void Pi05Runner::Release(Component c, double * context_ms)
{
  const size_t i = static_cast<size_t>(c);
  if (resident_[i] || mgrs_[i] == nullptr) {
    return;
  }
  const double start = NowMs();
  mgrs_[i].reset();
  *context_ms += NowMs() - start;
}

bool Pi05Runner::RunComponent(Component c,
    const std::vector<uint8_t> & inputs,
    std::unordered_map<std::string, std::vector<uint8_t>> * outputs,
    double * elapsed_ms,
    std::string * error)
{
  Mgr * mgr = mgrs_[static_cast<size_t>(c)].get();
  if (mgr == nullptr) {
    *error = std::string(ComponentName(c)) + ": context not acquired";
    return false;
  }
  const size_t ci = static_cast<size_t>(c);
  const int call_index = call_counts_[ci]++;

  const double start = NowMs();
  if (!mgr->inference_execute(inputs)) {
    *error = std::string(ComponentName(c)) + ": inference_execute failed";
    return false;
  }
  auto tensors = mgr->get_output_tensors();
  *elapsed_ms += NowMs() - start;
  if (tensors.empty()) {
    *error = std::string(ComponentName(c)) + ": no output tensors";
    return false;
  }
  // A short output vector means the context was created against a graph whose
  // arity differs from the committed header -- which upstream reports as a log
  // line rather than a failed constructor. Catch it here instead of trusting it.
  const size_t expected_outputs = ExpectedOutputCount(c);
  if (tensors.size() != expected_outputs) {
    std::ostringstream os;
    os << ComponentName(c) << ": graph returned " << tensors.size() << " output tensors, header "
       << "declares " << expected_outputs
       << " -- the context binary and pi05_graph_order.hpp disagree, or context creation "
          "silently failed (check for 'err 1002' / 'Failed to map weights buffer' above)";
    *error = os.str();
    return false;
  }

  if (!dumped_ && !options_.dump_dir.empty()) {
    const std::string base =
        options_.dump_dir + "/" + ComponentName(c) + "_call" + std::to_string(call_index);
    DumpRaw(base + "_IN.raw", inputs.data(), inputs.size());
    for (const auto & t : tensors) {
      DumpRaw(base + "_OUT_" + t.output_tensor_name + ".raw", t.output_tensor_data.data(),
          t.output_tensor_data.size());
    }
  }

  outputs->clear();
  outputs->reserve(tensors.size());
  for (auto & t : tensors) {
    (*outputs)[t.output_tensor_name] = std::move(t.output_tensor_data);
  }
  return true;
}

bool Pi05Runner::Infer(const Pi05Observation & obs, Pi05ActionChunk * out, std::string * error)
{
  if (!loaded_) {
    *error = "Pi05Runner::Load() has not been called";
    return false;
  }
  if (obs.images.size() > static_cast<size_t>(kNumCameras)) {
    *error = "more than " + std::to_string(kNumCameras) + " images supplied";
    return false;
  }
  if (obs.lang_tokens.size() != kMaxTokenLength || obs.lang_mask.size() != kMaxTokenLength) {
    *error = "lang_tokens and lang_mask must both be length " + std::to_string(kMaxTokenLength);
    return false;
  }
  for (const auto & img : obs.images) {
    if (img.size() != static_cast<size_t>(kImageElems)) {
      *error = "each image must be " + std::to_string(kImageElems) + " floats (3x224x224 CHW)";
      return false;
    }
  }

  Pi05Timings timings;
  timings.denoise_steps = options_.denoise_steps;
  const double t_total_start = NowMs();
  std::unordered_map<std::string, std::vector<uint8_t>> stage_out;

  // ---- Stage 1: vision_encoder, once per camera slot -----------------------
  // Empty slots are encoded as zero images rather than skipped: token_emb's
  // graph has a fixed arity of kNumCameras embeddings.
  std::vector<std::vector<uint8_t>> img_embeds(kNumCameras);
  if (!Acquire(Component::kVisionEncoder, &timings.context_ms, error)) {
    return false;
  }
  for (int cam = 0; cam < kNumCameras; ++cam) {
    const double t_pack = NowMs();
    if (static_cast<size_t>(cam) < obs.images.size()) {
      std::memcpy(vision_in_.data(), obs.images[cam].data(), vision_in_.size());
    } else {
      std::memset(vision_in_.data(), 0, vision_in_.size());
    }
    timings.pack_ms += NowMs() - t_pack;

    if (!RunComponent(Component::kVisionEncoder, vision_in_, &stage_out, &timings.vision_ms,
            error)) {
      return false;
    }
    auto it = stage_out.find(pi05::kVisionEncoderOutputs[0].name);
    if (it == stage_out.end()) {
      *error = "vision_encoder did not emit img_embed";
      return false;
    }
    img_embeds[cam] = std::move(it->second);
  }
  Release(Component::kVisionEncoder, &timings.context_ms);

  // ---- Stage 2: token_emb -------------------------------------------------
  {
    const double t_pack = NowMs();
    const auto offsets = OffsetMap(pi05::kTokenEmbInputs);
    for (const auto & spec : pi05::kTokenEmbInputs) {
      uint8_t * dst = token_in_.data() + offsets.at(spec.name);
      const std::string name(spec.name);
      if (name == "lang_tokens") {
        std::memcpy(dst, obs.lang_tokens.data(), spec.byte_count);
      } else if (name == "lang_mask") {
        std::memcpy(dst, obs.lang_mask.data(), spec.byte_count);
      } else if (name.rfind("img_embed", 0) == 0) {
        const int idx = std::stoi(name.substr(std::strlen("img_embed"))) - 1;
        if (idx < 0 || idx >= kNumCameras || img_embeds[idx].size() != spec.byte_count) {
          *error = "cannot satisfy token_emb input '" + name + "'";
          return false;
        }
        std::memcpy(dst, img_embeds[idx].data(), spec.byte_count);
      } else {
        *error = "unhandled token_emb input '" + name + "'";
        return false;
      }
    }
    timings.pack_ms += NowMs() - t_pack;
  }
  if (!Acquire(Component::kTokenEmb, &timings.context_ms, error)) {
    return false;
  }
  std::unordered_map<std::string, std::vector<uint8_t>> prefix;
  if (!RunComponent(Component::kTokenEmb, token_in_, &prefix, &timings.token_emb_ms, error)) {
    return false;
  }
  Release(Component::kTokenEmb, &timings.context_ms);

  // ---- Stage 3: backbone prefill -> 18 layers of K/V cache ----------------
  {
    const double t_pack = NowMs();
    const auto offsets = OffsetMap(pi05::kBackboneInputs);
    for (const auto & spec : pi05::kBackboneInputs) {
      const std::string src = BackboneInputSource(spec.name);
      if (src.empty()) {
        *error = std::string("unhandled backbone input '") + spec.name + "'";
        return false;
      }
      if (!CopyNamed(prefix, src, backbone_in_.data() + offsets.at(spec.name), spec.byte_count,
              error)) {
        return false;
      }
    }
    timings.pack_ms += NowMs() - t_pack;
  }
  if (!Acquire(Component::kBackbone, &timings.context_ms, error)) {
    return false;
  }
  std::unordered_map<std::string, std::vector<uint8_t>> kv;
  if (!RunComponent(Component::kBackbone, backbone_in_, &kv, &timings.backbone_ms, error)) {
    return false;
  }
  Release(Component::kBackbone, &timings.context_ms);

  // ---- Stage 4: action expert, Euler flow-matching integration ------------
  // Everything except x_t and time_step is constant across denoise steps, so
  // the ~34 MB KV block is packed once and only 6404 bytes are rewritten per
  // step. Mirrors app.py Pi05App.sample_action().
  const auto expert_offsets = OffsetMap(pi05::kActionExpertInputs);
  {
    const double t_pack = NowMs();
    for (const auto & spec : pi05::kActionExpertInputs) {
      const std::string name(spec.name);
      if (name == "x_t" || name == "time_step") {
        continue;  // written inside the loop
      }
      const std::string src = ExpertInputSource(name);
      if (src.empty()) {
        *error = "unhandled action_expert input '" + name + "'";
        return false;
      }
      const auto & pool = (src == "full_att_4d" || src.rfind("suffix_", 0) == 0) ? prefix : kv;
      if (!CopyNamed(pool, src, expert_in_.data() + expert_offsets.at(name), spec.byte_count,
              error)) {
        return false;
      }
    }
    timings.pack_ms += NowMs() - t_pack;
  }

  const size_t x_off = expert_offsets.at("x_t");
  const size_t t_off = expert_offsets.at("time_step");

  std::mt19937_64 rng(rng_state_);
  std::normal_distribution<float> gauss(0.0F, 1.0F);
  std::vector<float> x_t(static_cast<size_t>(kActionSteps) * kActionDim);
  for (auto & v : x_t) {
    v = gauss(rng);
  }
  rng_state_ = rng();

  if (!Acquire(Component::kActionExpert, &timings.context_ms, error)) {
    return false;
  }

  // dt is negative: integrate t from 1.0 down towards 0. The expert applies the
  // Euler update internally and returns x_{t+dt} directly, so there is no
  // host-side x += dt * v.
  const float dt = -1.0F / static_cast<float>(options_.denoise_steps);
  float t_cur = 1.0F;
  std::unordered_map<std::string, std::vector<uint8_t>> expert_out;
  for (int step = 0; step < options_.denoise_steps; ++step) {
    const double t_pack = NowMs();
    std::memcpy(expert_in_.data() + x_off, x_t.data(), x_t.size() * sizeof(float));
    std::memcpy(expert_in_.data() + t_off, &t_cur, sizeof(float));
    timings.pack_ms += NowMs() - t_pack;

    if (!RunComponent(Component::kActionExpert, expert_in_, &expert_out,
            &timings.action_expert_ms, error)) {
      return false;
    }
    if (!CopyNamed(expert_out, pi05::kActionExpertOutputs[0].name,
            reinterpret_cast<uint8_t *>(x_t.data()), x_t.size() * sizeof(float), error)) {
      return false;
    }
    t_cur += dt;
  }
  Release(Component::kActionExpert, &timings.context_ms);

  timings.total_ms = NowMs() - t_total_start;
  if (!dumped_ && !options_.dump_dir.empty()) {
    DumpRaw(options_.dump_dir + "/final_actions.raw", x_t.data(), x_t.size() * sizeof(float));
    dumped_ = true;
  }
  out->actions = std::move(x_t);
  out->timings = timings;
  return true;
}

}  // namespace qrb_ros_vla
