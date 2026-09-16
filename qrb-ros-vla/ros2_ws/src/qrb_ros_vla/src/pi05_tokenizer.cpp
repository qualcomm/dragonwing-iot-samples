// Copyright (c) 2026 QRB ROS VLA contributors.
// SPDX-License-Identifier: BSD-3-Clause

#include "qrb_ros_vla/pi05_tokenizer.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "sentencepiece_processor.h"

namespace qrb_ros_vla
{
namespace
{

// prompt.strip().replace("_", " ").replace("\n", " ")
std::string CleanTask(const std::string & task)
{
  const auto first = task.find_first_not_of(" \t\n\r\f\v");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = task.find_last_not_of(" \t\n\r\f\v");
  std::string cleaned = task.substr(first, last - first + 1);
  std::replace(cleaned.begin(), cleaned.end(), '_', ' ');
  std::replace(cleaned.begin(), cleaned.end(), '\n', ' ');
  return cleaned;
}

}  // namespace

Pi05Tokenizer::Pi05Tokenizer() = default;
Pi05Tokenizer::~Pi05Tokenizer() = default;

bool Pi05Tokenizer::loaded() const
{
  return sp_ != nullptr;
}

int Pi05Tokenizer::DiscretizeState(float value)
{
  // bins[i] = -1 + i * (2/256) for i in 0..255, and np.digitize with right=False
  // counts how many bin edges are <= value. Subtracting 1 turns that count into
  // a 0-based bin index, which reduces to floor((value + 1) * 128).
  if (!std::isfinite(value)) {
    return 128;  // mid-range; a NaN in the prompt would poison the whole chunk
  }
  const float scaled = (value + 1.0F) * 128.0F;
  const int index = static_cast<int>(std::floor(scaled));
  return std::clamp(index, 0, 255);
}

std::string Pi05Tokenizer::BuildPrompt(const std::string & task,
    const std::vector<float> & normalized_state) const
{
  const std::string cleaned = CleanTask(task);
  if (normalized_state.empty()) {
    // Pi0-style prompt: the trailing newline is the "start of answer" token.
    return cleaned + "\n";
  }
  std::ostringstream os;
  os << "Task: " << cleaned << ", State: ";
  for (size_t i = 0; i < normalized_state.size(); ++i) {
    if (i != 0) {
      os << ' ';
    }
    os << DiscretizeState(normalized_state[i]);
  }
  os << ";\nAction: ";
  return os.str();
}

bool Pi05Tokenizer::Load(const std::string & model_path, int max_tokens, std::string * error)
{
  if (max_tokens <= 0) {
    *error = "max_tokens must be positive";
    return false;
  }
  auto sp = std::make_unique<sentencepiece::SentencePieceProcessor>();
  const auto status = sp->Load(model_path);
  if (!status.ok()) {
    *error = "cannot load SentencePiece model '" + model_path + "': " + status.ToString() +
             " (fetch it with scripts/fetch-tokenizer.sh)";
    return false;
  }
  // openpi passes add_bos=True; in the C++ API that is an encode extra option.
  const auto extra = sp->SetEncodeExtraOptions("bos");
  if (!extra.ok()) {
    *error = "cannot enable BOS on the tokenizer: " + extra.ToString();
    return false;
  }
  sp_ = std::move(sp);
  max_tokens_ = max_tokens;
  return true;
}

bool Pi05Tokenizer::Encode(const std::string & task,
    const std::vector<float> & normalized_state,
    std::vector<int32_t> * tokens,
    std::vector<float> * mask,
    int * real_tokens,
    std::string * error) const
{
  if (sp_ == nullptr) {
    *error = "tokenizer not loaded";
    return false;
  }

  const std::string prompt = BuildPrompt(task, normalized_state);
  std::vector<int> ids;
  const auto status = sp_->Encode(prompt, &ids);
  if (!status.ok()) {
    *error = "tokenization failed: " + status.ToString();
    return false;
  }

  const size_t limit = static_cast<size_t>(max_tokens_);
  const bool truncated = ids.size() > limit;
  if (truncated) {
    // Truncating silently would quietly drop the end of the prompt -- which for
    // Pi0.5 is the "Action:" cue and part of the state -- so surface it.
    ids.resize(limit);
  }

  tokens->assign(limit, 0);
  mask->assign(limit, 0.0F);
  for (size_t i = 0; i < ids.size(); ++i) {
    (*tokens)[i] = static_cast<int32_t>(ids[i]);
    (*mask)[i] = 1.0F;
  }
  *real_tokens = static_cast<int>(ids.size());

  if (truncated) {
    *error = "prompt tokenized to more than " + std::to_string(max_tokens_) +
             " tokens and was truncated; shorten the task text or reduce the state dimension";
    return false;
  }
  return true;
}

}  // namespace qrb_ros_vla
