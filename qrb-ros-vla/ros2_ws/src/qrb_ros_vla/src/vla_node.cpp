// Copyright (c) 2026 QRB ROS VLA contributors.
// SPDX-License-Identifier: BSD-3-Clause
//
// VlaNode: ROS 2 front-end for Pi0.5 on the IQ-9075 NPU.
//
//   in : N x sensor_msgs/Image        (camera streams, any size/encoding)
//        std_msgs/String  ~/task     (instruction; resolved via the phrasebook)
//        std_msgs/Int32MultiArray ~/lang_tokens  (pre-tokenized override)
//   out: qrb_ros_vla_msgs/ActionChunk    ~/action_chunk
//        qrb_ros_vla_msgs/InferenceStats ~/stats
//
// Inference costs ~1 s per chunk, which is far too long to run on an executor
// thread, so it runs on a dedicated worker. Frames that arrive mid-inference are
// dropped rather than queued: a VLA acting on stale observations is worse than
// one acting at a lower rate.
//
// Tokenization deliberately does NOT happen here. Pi0.5 uses the PaliGemma
// tokenizer and folds discretized robot state into the prompt, so token ids come
// from a phrasebook generated offline by scripts/tokenize_prompts.py. That keeps
// the on-device path free of a gated tokenizer download and works offline.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/imgproc.hpp"
#include "qrb_ros_vla/pi05_runner.hpp"
#include "qrb_ros_vla/pi05_tokenizer.hpp"
#include "qrb_ros_vla_msgs/msg/action_chunk.hpp"
#include "qrb_ros_vla_msgs/msg/inference_stats.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace qrb_ros_vla
{
namespace
{

struct PhraseEntry
{
  std::vector<int32_t> tokens;
  int real_tokens = 0;
};

// Phrasebook format, one task per line, tab separated:
//   <task text> \t <count of real tokens> \t <200 comma-separated token ids>
// Lines starting with '#' are comments. Deliberately dependency-free: no JSON
// parser in the on-device path.
bool LoadPhrasebook(const std::string & path,
    std::unordered_map<std::string, PhraseEntry> * out,
    std::string * error)
{
  std::ifstream in(path);
  if (!in) {
    *error = "cannot open phrasebook: " + path;
    return false;
  }
  std::string line;
  size_t lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const size_t tab1 = line.find('\t');
    const size_t tab2 = line.find('\t', tab1 == std::string::npos ? 0 : tab1 + 1);
    if (tab1 == std::string::npos || tab2 == std::string::npos) {
      *error = path + ":" + std::to_string(lineno) + ": expected two tab separators";
      return false;
    }
    PhraseEntry entry;
    entry.real_tokens = std::atoi(line.substr(tab1 + 1, tab2 - tab1 - 1).c_str());
    std::stringstream ids(line.substr(tab2 + 1));
    std::string tok;
    while (std::getline(ids, tok, ',')) {
      if (!tok.empty()) {
        entry.tokens.push_back(static_cast<int32_t>(std::strtol(tok.c_str(), nullptr, 10)));
      }
    }
    if (entry.tokens.size() != static_cast<size_t>(kMaxTokenLength)) {
      *error = path + ":" + std::to_string(lineno) + ": got " +
               std::to_string(entry.tokens.size()) + " token ids, expected " +
               std::to_string(kMaxTokenLength);
      return false;
    }
    (*out)[line.substr(0, tab1)] = std::move(entry);
  }
  if (out->empty()) {
    *error = "phrasebook " + path + " contained no entries";
    return false;
  }
  return true;
}

// Mirrors qai_hub_models.utils.image_processing.resize_and_normalize:
// aspect-preserving resize into 224x224 anchored top-left, zero padding for the
// remainder, then scale [0,1] -> [-1,1]. Output is CHW float.
void ResizeNormalizeChw(const cv::Mat & rgb8, std::vector<float> * out)
{
  const double scale =
      std::min(static_cast<double>(kImageH) / rgb8.rows, static_cast<double>(kImageW) / rgb8.cols);
  const int new_h = std::max(1, std::min(kImageH, static_cast<int>(std::round(rgb8.rows * scale))));
  const int new_w = std::max(1, std::min(kImageW, static_cast<int>(std::round(rgb8.cols * scale))));

  cv::Mat resized;
  cv::resize(rgb8, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  out->assign(static_cast<size_t>(kImageElems), -1.0F);  // zero in [0,1] == -1 after scaling
  const size_t plane = static_cast<size_t>(kImageH) * kImageW;
  for (int y = 0; y < new_h; ++y) {
    const uint8_t * row = resized.ptr<uint8_t>(y);
    for (int x = 0; x < new_w; ++x) {
      for (int c = 0; c < 3; ++c) {
        const float v = static_cast<float>(row[x * 3 + c]) / 255.0F;
        (*out)[c * plane + static_cast<size_t>(y) * kImageW + x] = v * 2.0F - 1.0F;
      }
    }
  }
}

}  // namespace

class VlaNode : public rclcpp::Node
{
public:
  explicit VlaNode(const rclcpp::NodeOptions & options) : rclcpp::Node("qrb_ros_vla", options)
  {
    const auto bundle = declare_parameter<std::string>("bundle_dir", "");
    const auto backend = declare_parameter<std::string>("backend", "libQnnHtp.so");
    const auto steps = declare_parameter<int>("denoise_steps", kDefaultDenoiseSteps);
    const auto seed = declare_parameter<int>("seed", 0);
    const auto phrasebook = declare_parameter<std::string>("phrasebook", "");
    const auto default_task = declare_parameter<std::string>("default_task", "");
    const auto tokenizer_model = declare_parameter<std::string>("tokenizer_model", "");
    action_dof_ = declare_parameter<int>("action_dof", 7);
    // Which HTP device each component runs on, in Component order:
    // vision_encoder, token_emb, backbone, action_expert. QCS9075 has two NPUs
    // and device 0 is the faster one, so the default puts the expensive pair
    // there. Exposed as a parameter so the placement can be explored without
    // rebuilding.
    const auto device_ids = declare_parameter<std::vector<int64_t>>(
        "htp_device_ids", std::vector<int64_t>{1, 1, 0, 0});
    // Components kept mapped on the NPU for the process lifetime. All four fits
    // only because they are split across both devices.
    const auto resident = declare_parameter<std::vector<std::string>>(
        "resident", std::vector<std::string>{
                        "vision_encoder", "token_emb", "backbone", "action_expert"});
    camera_topics_ = declare_parameter<std::vector<std::string>>(
        "camera_topics", std::vector<std::string>{"/vla/camera0/image_raw", "/vla/camera1/image_raw"});

    if (bundle.empty()) {
      throw std::runtime_error("parameter 'bundle_dir' is required");
    }
    if (camera_topics_.empty() || camera_topics_.size() > static_cast<size_t>(kNumCameras)) {
      throw std::runtime_error(
          "camera_topics must name 1.." + std::to_string(kNumCameras) + " topics");
    }

    if (!phrasebook.empty()) {
      std::string error;
      if (!LoadPhrasebook(phrasebook, &phrasebook_, &error)) {
        throw std::runtime_error(error);
      }
      RCLCPP_INFO(get_logger(), "phrasebook: %zu tasks from %s", phrasebook_.size(),
          phrasebook.c_str());
    }
    // Live tokenization is the preferred path: Pi0.5 splices discretized state
    // into the prompt, so a static phrasebook can only ever be correct for one
    // state. The phrasebook remains as an offline fallback.
    if (!tokenizer_model.empty()) {
      std::string error;
      if (!tokenizer_.Load(tokenizer_model, kMaxTokenLength, &error)) {
        throw std::runtime_error(error);
      }
      RCLCPP_INFO(get_logger(), "tokenizer: %s (live prompt build, state folded in)",
          tokenizer_model.c_str());
    } else if (phrasebook.empty()) {
      RCLCPP_WARN(get_logger(),
          "neither 'tokenizer_model' nor 'phrasebook' is set; the node will only act on raw ids "
          "published to ~/lang_tokens");
    }

    Pi05Runner::Options opts;
    opts.bundle_dir = bundle;
    opts.backend = backend;
    opts.denoise_steps = steps;
    opts.seed = static_cast<uint64_t>(seed);
    opts.resident = resident;
    opts.device_ids.clear();
    for (const auto id : device_ids) {
      opts.device_ids.push_back(static_cast<uint32_t>(id));
    }
    if (opts.device_ids.size() != static_cast<size_t>(Component::kCount)) {
      throw std::runtime_error("'htp_device_ids' needs exactly " +
                               std::to_string(static_cast<int>(Component::kCount)) +
                               " entries (vision_encoder, token_emb, backbone, action_expert)");
    }
    runner_ = std::make_unique<Pi05Runner>(opts);
    backend_ = backend;

    latest_images_.resize(camera_topics_.size());
    have_image_.assign(camera_topics_.size(), false);

    chunk_pub_ = create_publisher<qrb_ros_vla_msgs::msg::ActionChunk>("~/action_chunk", 10);
    stats_pub_ = create_publisher<qrb_ros_vla_msgs::msg::InferenceStats>("~/stats", 10);

    for (size_t i = 0; i < camera_topics_.size(); ++i) {
      auto cb = [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) { OnImage(i, msg); };
      image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
          camera_topics_[i], rclcpp::SensorDataQoS(), cb));
      RCLCPP_INFO(get_logger(), "camera %zu <- %s", i, camera_topics_[i].c_str());
    }
    task_sub_ = create_subscription<std_msgs::msg::String>("~/task", 10,
        [this](std_msgs::msg::String::ConstSharedPtr msg) { OnTask(msg->data); });
    tokens_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>("~/lang_tokens", 10,
        [this](std_msgs::msg::Int32MultiArray::ConstSharedPtr msg) { OnTokens(msg->data); });
    // Proprioceptive state, already normalized to ~[-1, 1] with the policy's
    // training statistics. Re-tokenizes on every update, which is ~0.1 ms.
    state_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>("~/state", 10,
        [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { OnState(msg->data); });

    if (!default_task.empty()) {
      OnTask(default_task);
    }

    worker_ = std::thread(&VlaNode::Worker, this);
    RCLCPP_INFO(get_logger(),
        "loading %s in background (backend %s, %d denoise steps) - inference starts once ready",
        bundle.c_str(), backend.c_str(), static_cast<int>(steps));
  }

  ~VlaNode() override
  {
    stop_ = true;
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void OnTask(const std::string & task)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    task_ = task;
    have_task_text_ = true;
    int real_tokens = 0;
    if (!RetokenizeLocked(&real_tokens)) {
      return;
    }
    RCLCPP_INFO(get_logger(), "task set: '%s' (%d real tokens, %zu state dims)", task.c_str(),
        real_tokens, state_.size());
  }

  void OnState(const std::vector<float> & state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
    if (!have_task_text_ || !tokenizer_.loaded()) {
      // Without a live tokenizer the state cannot reach the model at all: it is
      // only ever carried inside the prompt. Say so once instead of silently
      // ignoring the topic.
      if (!warned_state_ignored_ && !tokenizer_.loaded()) {
        warned_state_ignored_ = true;
        RCLCPP_WARN(get_logger(),
            "~/state received but no 'tokenizer_model' is set, so state cannot be encoded; "
            "Pi0.5 carries state in the prompt, not as a tensor");
      }
      return;
    }
    int real_tokens = 0;
    RetokenizeLocked(&real_tokens);
  }

  // Caller must hold mutex_. Prefers live tokenization; falls back to the
  // offline phrasebook, which ignores state by construction.
  bool RetokenizeLocked(int * real_tokens)
  {
    if (tokenizer_.loaded()) {
      std::string error;
      if (!tokenizer_.Encode(task_, state_, &tokens_, &mask_, real_tokens, &error)) {
        RCLCPP_ERROR(get_logger(), "tokenization failed: %s", error.c_str());
        return false;
      }
      return true;
    }
    if (phrasebook_.empty()) {
      RCLCPP_WARN(get_logger(),
          "received task '%s' but neither a tokenizer nor a phrasebook is loaded; publish token "
          "ids on ~/lang_tokens instead",
          task_.c_str());
      return false;
    }
    auto it = phrasebook_.find(task_);
    if (it == phrasebook_.end()) {
      RCLCPP_ERROR(get_logger(),
          "task '%s' is not in the phrasebook; regenerate it with scripts/tokenize_prompts.py "
          "or set 'tokenizer_model' to tokenize live",
          task_.c_str());
      return false;
    }
    tokens_ = it->second.tokens;
    mask_.assign(kMaxTokenLength, 0.0F);
    for (int i = 0; i < std::min(it->second.real_tokens, kMaxTokenLength); ++i) {
      mask_[i] = 1.0F;
    }
    *real_tokens = it->second.real_tokens;
    return true;
  }

  void OnTokens(const std::vector<int32_t> & data)
  {
    if (data.size() != static_cast<size_t>(kMaxTokenLength)) {
      RCLCPP_ERROR(get_logger(), "~/lang_tokens must carry exactly %d ids, got %zu",
          kMaxTokenLength, data.size());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    tokens_ = data;
    mask_.assign(kMaxTokenLength, 0.0F);
    for (int i = 0; i < kMaxTokenLength; ++i) {
      // PaliGemma pads on the right with id 0; treat the run of trailing zeros
      // as padding rather than trusting a separately-published mask.
      if (data[i] != 0) {
        mask_[i] = 1.0F;
      }
    }
    task_ = "<raw tokens>";
  }

  void OnImage(size_t index, const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat rgb;
    try {
      rgb = cv_bridge::toCvShare(msg, "rgb8")->image;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge failed on camera %zu: %s",
          index, e.what());
      return;
    }
    std::vector<float> chw;
    ResizeNormalizeChw(rgb, &chw);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_images_[index] = std::move(chw);
      have_image_[index] = true;
      last_stamp_ = msg->header.stamp;
      last_frame_id_ = msg->header.frame_id;
    }
    cv_.notify_one();
  }

  bool Ready() const
  {
    if (tokens_.size() != static_cast<size_t>(kMaxTokenLength)) {
      return false;
    }
    return std::all_of(have_image_.begin(), have_image_.end(), [](bool b) { return b; });
  }

  void Worker()
  {
    std::string error;
    if (!runner_->Load(&error)) {
      RCLCPP_FATAL(get_logger(), "failed to load pi0.5 bundle: %s", error.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "pi0.5 bundle loaded; waiting for %zu camera streams and a task",
        camera_topics_.size());

    while (!stop_) {
      Pi05Observation obs;
      builtin_interfaces::msg::Time stamp;
      std::string frame_id;
      std::string task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stop_ || Ready(); });
        if (stop_) {
          break;
        }
        obs.images = latest_images_;
        obs.lang_tokens = tokens_;
        obs.lang_mask = mask_;
        stamp = last_stamp_;
        frame_id = last_frame_id_;
        task = task_;
        // Require a fresh frame before the next chunk: acting twice on the same
        // observation would inflate the apparent rate without adding control.
        std::fill(have_image_.begin(), have_image_.end(), false);
      }

      Pi05ActionChunk chunk;
      if (!runner_->Infer(obs, &chunk, &error)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "inference failed: %s",
            error.c_str());
        continue;
      }

      qrb_ros_vla_msgs::msg::ActionChunk out;
      out.header.stamp = stamp;
      out.header.frame_id = frame_id;
      out.task = task;
      out.chunk_size = kActionSteps;
      out.action_dim = kActionDim;
      out.action_dof = static_cast<uint32_t>(action_dof_);
      out.actions = std::move(chunk.actions);
      out.unnormalized = false;
      out.denoise_steps = static_cast<uint32_t>(chunk.timings.denoise_steps);
      chunk_pub_->publish(out);

      qrb_ros_vla_msgs::msg::InferenceStats stats;
      stats.header.stamp = now();
      stats.header.frame_id = frame_id;
      stats.vision_encoder_ms = chunk.timings.vision_ms;
      stats.token_emb_ms = chunk.timings.token_emb_ms;
      stats.backbone_ms = chunk.timings.backbone_ms;
      stats.action_expert_ms = chunk.timings.action_expert_ms;
      stats.host_packing_ms = chunk.timings.pack_ms;
      stats.total_ms = chunk.timings.total_ms;
      stats.camera_passes = kNumCameras;
      stats.denoise_steps = static_cast<uint32_t>(chunk.timings.denoise_steps);
      stats.backend = backend_;
      stats.chunks_per_second =
          chunk.timings.total_ms > 0.0 ? 1000.0 / chunk.timings.total_ms : 0.0;
      stats_pub_->publish(stats);

      RCLCPP_INFO(get_logger(),
          "chunk: total %.1f ms (vision %.1f, token_emb %.1f, backbone %.1f, expert %.1f x%d, "
          "pack %.1f) -> %.2f chunks/s",
          chunk.timings.total_ms, chunk.timings.vision_ms, chunk.timings.token_emb_ms,
          chunk.timings.backbone_ms, chunk.timings.action_expert_ms,
          chunk.timings.denoise_steps, chunk.timings.pack_ms, stats.chunks_per_second);
    }
  }

  std::unique_ptr<Pi05Runner> runner_;
  std::string backend_;
  int action_dof_ = 7;
  std::vector<std::string> camera_topics_;
  std::unordered_map<std::string, PhraseEntry> phrasebook_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tokens_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
  Pi05Tokenizer tokenizer_;
  std::vector<float> state_;
  bool have_task_text_ = false;
  bool warned_state_ignored_ = false;
  rclcpp::Publisher<qrb_ros_vla_msgs::msg::ActionChunk>::SharedPtr chunk_pub_;
  rclcpp::Publisher<qrb_ros_vla_msgs::msg::InferenceStats>::SharedPtr stats_pub_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<std::vector<float>> latest_images_;
  std::vector<bool> have_image_;
  std::vector<int32_t> tokens_;
  std::vector<float> mask_;
  std::string task_;
  builtin_interfaces::msg::Time last_stamp_;
  std::string last_frame_id_;

  std::thread worker_;
  std::atomic<bool> stop_{false};
};

}  // namespace qrb_ros_vla

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros_vla::VlaNode)
