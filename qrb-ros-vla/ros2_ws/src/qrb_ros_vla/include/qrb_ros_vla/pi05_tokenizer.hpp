// Copyright (c) 2026 QRB ROS VLA contributors.
// SPDX-License-Identifier: BSD-3-Clause
//
// PaliGemma tokenization for Pi0.5, on device, in process.
//
// Pi0.5 does NOT take robot state as a tensor. It discretizes state into 256
// bins and splices it into the *language* prompt, so the prompt -- and therefore
// the token ids -- change on every control step. That rules out a precomputed
// phrasebook for closed-loop operation and is why this runs in the node.
//
// The prompt format is reproduced exactly from openpi's PaligemmaTokenizer
// (src/openpi/models/tokenizer.py, the `state is not None` branch):
//
//     cleaned = prompt.strip().replace("_", " ").replace("\n", " ")
//     discretized = np.digitize(state, np.linspace(-1, 1, 257)[:-1]) - 1
//     full = f"Task: {cleaned}, State: {' '.join(discretized)};\nAction: "
//     tokens = tokenizer.encode(full, add_bos=True)
//
// then right-padded with id 0 to max_token_length (200 for Pi0.5) with a
// parallel 1.0/0.0 mask.
//
// The tokenizer model itself is the standard PaliGemma SentencePiece model.
// google/paligemma-3b-pt-224 on Hugging Face is gated, but the same file is
// served without authentication from the big_vision bucket, which is where
// openpi gets it too:
//   https://storage.googleapis.com/big_vision/paligemma_tokenizer.model
// See scripts/fetch-tokenizer.sh.

#ifndef QRB_ROS_VLA__PI05_TOKENIZER_HPP_
#define QRB_ROS_VLA__PI05_TOKENIZER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sentencepiece
{
class SentencePieceProcessor;
}

namespace qrb_ros_vla
{

class Pi05Tokenizer
{
public:
  Pi05Tokenizer();
  ~Pi05Tokenizer();

  Pi05Tokenizer(const Pi05Tokenizer &) = delete;
  Pi05Tokenizer & operator=(const Pi05Tokenizer &) = delete;

  // max_tokens must match the exported token_emb graph (200 for Pi0.5).
  bool Load(const std::string & model_path, int max_tokens, std::string * error);
  bool loaded() const;

  // state must already be normalized to roughly [-1, 1] using the same dataset
  // statistics the policy was trained with; values outside that range are
  // clamped into the outermost bins. Pass an empty state for the Pi0-style
  // prompt (no "State:" clause).
  bool Encode(const std::string & task,
      const std::vector<float> & normalized_state,
      std::vector<int32_t> * tokens,
      std::vector<float> * mask,
      int * real_tokens,
      std::string * error) const;

  // Exposed for tests and for logging what the model was actually asked.
  std::string BuildPrompt(const std::string & task,
      const std::vector<float> & normalized_state) const;

  // np.digitize(x, np.linspace(-1, 1, 257)[:-1]) - 1, clamped to [0, 255].
  static int DiscretizeState(float value);

private:
  std::unique_ptr<sentencepiece::SentencePieceProcessor> sp_;
  int max_tokens_ = 0;
};

}  // namespace qrb_ros_vla

#endif  // QRB_ROS_VLA__PI05_TOKENIZER_HPP_
