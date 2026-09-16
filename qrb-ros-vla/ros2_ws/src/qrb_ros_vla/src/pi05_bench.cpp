// Copyright (c) 2026 QRB ROS VLA contributors.
// SPDX-License-Identifier: BSD-3-Clause
//
// Standalone latency harness for the Pi0.5 NPU pipeline. No ROS graph involved,
// so a number produced here is the model cost alone. Every published latency
// figure in this repo must come from this binary (or the node's own stats
// topic), never from an estimate.
//
//   pi05_bench --bundle <dir> [--iters 20] [--warmup 3] [--steps 10]
//              [--backend libQnnHtp.so] [--json out.json] [--seed 1]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "qrb_ros_vla/pi05_runner.hpp"

namespace
{

struct Args
{
  std::string bundle;
  std::string backend = "libQnnHtp.so";
  std::string json_path;
  std::string dump_dir;
  // Comma-separated component names kept mapped on the NPU; "" means rotate all.
  std::string resident = "vision_encoder,token_emb,backbone,action_expert";
  // HTP device per component, in Component order: vision,token_emb,backbone,expert.
  std::string devices = "1,1,0,0";
  int iters = 20;
  int warmup = 3;
  int steps = qrb_ros_vla::kDefaultDenoiseSteps;
  uint64_t seed = 1;
};

std::vector<std::string> SplitCsv(const std::string & csv)
{
  std::vector<std::string> parts;
  std::string item;
  std::stringstream ss(csv);
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) {
      parts.push_back(item);
    }
  }
  return parts;
}

bool ParseArgs(int argc, char ** argv, Args * args)
{
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    auto next = [&]() -> const char * { return (i + 1 < argc) ? argv[++i] : nullptr; };
    if (flag == "--bundle") {
      const char * v = next();
      if (!v) return false;
      args->bundle = v;
    } else if (flag == "--backend") {
      const char * v = next();
      if (!v) return false;
      args->backend = v;
    } else if (flag == "--json") {
      const char * v = next();
      if (!v) return false;
      args->json_path = v;
    } else if (flag == "--devices") {
      const char * v = next();
      if (!v) return false;
      args->devices = v;
    } else if (flag == "--dump-dir") {
      const char * v = next();
      if (!v) return false;
      args->dump_dir = v;
    } else if (flag == "--resident") {
      const char * v = next();
      if (!v) return false;
      args->resident = v;
    } else if (flag == "--iters") {
      const char * v = next();
      if (!v) return false;
      args->iters = std::atoi(v);
    } else if (flag == "--warmup") {
      const char * v = next();
      if (!v) return false;
      args->warmup = std::atoi(v);
    } else if (flag == "--steps") {
      const char * v = next();
      if (!v) return false;
      args->steps = std::atoi(v);
    } else if (flag == "--seed") {
      const char * v = next();
      if (!v) return false;
      args->seed = std::strtoull(v, nullptr, 10);
    } else {
      std::cerr << "unknown flag: " << flag << "\n";
      return false;
    }
  }
  return !args->bundle.empty() && args->iters > 0 && args->steps > 0;
}

struct Stats
{
  double mean = 0.0;
  double p50 = 0.0;
  double p95 = 0.0;
  double min = 0.0;
  double max = 0.0;
  double stddev = 0.0;
};

Stats Summarize(std::vector<double> v)
{
  Stats s;
  if (v.empty()) {
    return s;
  }
  std::sort(v.begin(), v.end());
  s.min = v.front();
  s.max = v.back();
  s.mean = std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
  s.p50 = v[v.size() / 2];
  s.p95 = v[std::min(v.size() - 1, static_cast<size_t>(std::llround(0.95 * (v.size() - 1))))];
  double acc = 0.0;
  for (double x : v) {
    acc += (x - s.mean) * (x - s.mean);
  }
  s.stddev = std::sqrt(acc / static_cast<double>(v.size()));
  return s;
}

void PrintRow(const char * label, const Stats & s, int calls_per_chunk)
{
  std::cout << "  " << std::left << std::setw(18) << label << std::right << std::fixed
            << std::setprecision(2) << std::setw(10) << s.mean << std::setw(10) << s.p50
            << std::setw(10) << s.p95 << std::setw(10) << s.min << std::setw(10) << s.max
            << std::setw(9) << calls_per_chunk << "\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  if (!ParseArgs(argc, argv, &args)) {
    std::cerr << "usage: pi05_bench --bundle <dir> [--iters N] [--warmup N] [--steps N]\n"
                 "                  [--backend libQnnHtp.so] [--json out.json] [--seed N]\n";
    return 2;
  }

  qrb_ros_vla::Pi05Runner::Options opts;
  opts.bundle_dir = args.bundle;
  opts.backend = args.backend;
  opts.denoise_steps = args.steps;
  opts.seed = args.seed;
  opts.resident = SplitCsv(args.resident);
  opts.dump_dir = args.dump_dir;
  {
    std::vector<uint32_t> ids;
    for (const auto & s : SplitCsv(args.devices)) {
      ids.push_back(static_cast<uint32_t>(std::stoul(s)));
    }
    if (ids.size() != static_cast<size_t>(qrb_ros_vla::Component::kCount)) {
      std::cerr << "--devices needs " << static_cast<int>(qrb_ros_vla::Component::kCount)
                << " comma-separated ids (vision_encoder,token_emb,backbone,action_expert)\n";
      return 2;
    }
    opts.device_ids = ids;
  }

  qrb_ros_vla::Pi05Runner runner(opts);
  std::cout << "residency: " << (args.resident.empty() ? "<rotate all>" : args.resident)
            << "  |  htp devices (vision,token_emb,backbone,expert): " << args.devices << "\n";
  std::cout << "loading pi0.5 context binaries from " << args.bundle << " (backend "
            << args.backend << ") ...\n"
            << std::flush;

  const auto load_start = std::chrono::steady_clock::now();
  std::string error;
  if (!runner.Load(&error)) {
    std::cerr << "FAILED to load: " << error << "\n";
    return 1;
  }
  const double load_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - load_start)
          .count();
  std::cout << "loaded in " << std::fixed << std::setprecision(1) << load_ms / 1000.0 << " s\n";

  // Synthetic but correctly-shaped observation. Values live in the range the
  // vision encoder was exported for; this measures cost, not task accuracy.
  qrb_ros_vla::Pi05Observation obs;
  obs.images.resize(2, std::vector<float>(qrb_ros_vla::kImageElems));
  for (size_t c = 0; c < obs.images.size(); ++c) {
    for (int i = 0; i < qrb_ros_vla::kImageElems; ++i) {
      obs.images[c][i] = std::sin(static_cast<float>(i + c * 7919) * 0.001F);
    }
  }
  obs.lang_tokens.assign(qrb_ros_vla::kMaxTokenLength, 0);
  obs.lang_mask.assign(qrb_ros_vla::kMaxTokenLength, 0.0F);
  const int kFakeTokens = 24;
  for (int i = 0; i < kFakeTokens; ++i) {
    obs.lang_tokens[i] = 1000 + i;
    obs.lang_mask[i] = 1.0F;
  }

  std::vector<double> total, vision, token_emb, backbone, expert, pack, context;
  qrb_ros_vla::Pi05ActionChunk chunk;

  for (int i = 0; i < args.warmup + args.iters; ++i) {
    if (!runner.Infer(obs, &chunk, &error)) {
      std::cerr << "FAILED at iteration " << i << ": " << error << "\n";
      return 1;
    }
    const bool measured = i >= args.warmup;
    if (i == 0) {
      // Sanity: a chunk of exact zeros or NaNs means the graph ran but the
      // wiring is wrong, which is the failure mode worth catching early.
      bool all_zero = true;
      bool any_nan = false;
      for (float v : chunk.actions) {
        if (v != 0.0F) all_zero = false;
        if (std::isnan(v)) any_nan = true;
      }
      std::cout << "first chunk: " << chunk.actions.size() << " values, all_zero=" << all_zero
                << " any_nan=" << any_nan << "\n";
      if (all_zero || any_nan) {
        std::cerr << "FAILED: action chunk is degenerate (all-zero or NaN)\n";
        return 1;
      }
    }
    if (measured) {
      total.push_back(chunk.timings.total_ms);
      vision.push_back(chunk.timings.vision_ms);
      token_emb.push_back(chunk.timings.token_emb_ms);
      backbone.push_back(chunk.timings.backbone_ms);
      expert.push_back(chunk.timings.action_expert_ms);
      pack.push_back(chunk.timings.pack_ms);
      context.push_back(chunk.timings.context_ms);
    }
    std::cout << (measured ? "." : "w") << std::flush;
  }
  std::cout << "\n\n";

  const auto s_total = Summarize(total);
  const auto s_vision = Summarize(vision);
  const auto s_token = Summarize(token_emb);
  const auto s_backbone = Summarize(backbone);
  const auto s_expert = Summarize(expert);
  const auto s_pack = Summarize(pack);
  const auto s_context = Summarize(context);

  std::cout << "pi0.5 action-chunk latency, " << args.iters << " iters after " << args.warmup
            << " warmup, " << args.steps << " denoise steps\n"
            << "  " << std::left << std::setw(18) << "stage (ms)" << std::right << std::setw(10)
            << "mean" << std::setw(10) << "p50" << std::setw(10) << "p95" << std::setw(10) << "min"
            << std::setw(10) << "max" << std::setw(9) << "calls" << "\n";
  PrintRow("vision_encoder", s_vision, qrb_ros_vla::kNumCameras);
  PrintRow("token_emb", s_token, 1);
  PrintRow("backbone", s_backbone, 1);
  PrintRow("action_expert", s_expert, args.steps);
  PrintRow("host packing", s_pack, 0);
  PrintRow("ctx create/free", s_context, 0);
  PrintRow("TOTAL / chunk", s_total, 1);

  const double chunk_hz = s_total.mean > 0.0 ? 1000.0 / s_total.mean : 0.0;
  std::cout << "\n  chunks/s: " << std::fixed << std::setprecision(3) << chunk_hz
            << "   actions/chunk: " << qrb_ros_vla::kActionSteps << "\n";

  if (!args.json_path.empty()) {
    std::ofstream out(args.json_path);
    if (!out) {
      std::cerr << "could not write " << args.json_path << "\n";
      return 1;
    }
    auto emit = [&out](const char * name, const Stats & s, bool comma) {
      out << "    \"" << name << "\": {\"mean\": " << s.mean << ", \"p50\": " << s.p50
          << ", \"p95\": " << s.p95 << ", \"min\": " << s.min << ", \"max\": " << s.max
          << ", \"stddev\": " << s.stddev << "}" << (comma ? "," : "") << "\n";
    };
    out << std::fixed << std::setprecision(4);
    out << "{\n  \"bundle\": \"" << args.bundle << "\",\n"
        << "  \"backend\": \"" << args.backend << "\",\n"
        << "  \"resident\": \"" << args.resident << "\",\n"
        << "  \"htp_device_ids\": \"" << args.devices << "\",\n"
        << "  \"denoise_steps\": " << args.steps << ",\n"
        << "  \"iters\": " << args.iters << ",\n"
        << "  \"warmup\": " << args.warmup << ",\n"
        << "  \"load_seconds\": " << load_ms / 1000.0 << ",\n"
        << "  \"chunks_per_second\": " << chunk_hz << ",\n"
        << "  \"stage_ms\": {\n";
    emit("vision_encoder_all_cameras", s_vision, true);
    emit("token_emb", s_token, true);
    emit("backbone", s_backbone, true);
    emit("action_expert_all_steps", s_expert, true);
    emit("host_packing", s_pack, true);
    emit("context_create_free", s_context, true);
    emit("total_per_chunk", s_total, false);
    out << "  }\n}\n";
    std::cout << "  wrote " << args.json_path << "\n";
  }
  return 0;
}
