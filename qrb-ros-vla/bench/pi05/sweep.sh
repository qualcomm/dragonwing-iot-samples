#!/usr/bin/env bash
set -uo pipefail
export LD_LIBRARY_PATH=/usr/lib ADSP_LIBRARY_PATH=/usr/lib/dsp/cdsp
ART=/home/ubuntu/QRB-ROS-VLA/artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075
cd /home/ubuntu/QRB-ROS-VLA/bench/pi05
run() { # tag graph n perf extra...
  local tag=$1 g=$2 n=$3 pp=$4; shift 4
  local o=/tmp/sw_${tag}; rm -rf "$o"
  local t0 t1; t0=$(date +%s.%N)
  qnn-net-run --backend /usr/lib/libQnnHtp.so --retrieve_context "$ART/$g.bin" \
    --input_list "$g.inputlist" --output_dir "$o" --num_inferences "$n" \
    --perf_profile "$pp" --keep_num_outputs 1 --profiling_level basic "$@" \
    >/tmp/sw_${tag}.stdout 2>&1
  local rc=$?; t1=$(date +%s.%N)
  local p='{}'; [ -f "$o/qnn-profiling-data_0.log" ] && p=$(python3 parse_prof.py "$o/qnn-profiling-data_0.log")
  echo "{\"tag\":\"$tag\",\"graph\":\"$g\",\"n\":$n,\"perf\":\"$pp\",\"rc\":$rc,\"wall_s\":$(echo "$t1-$t0"|bc),\"args\":\"$*\",\"prof\":$p}"
}
for g in vision_encoder token_emb backbone action_expert; do
  for pp in default burst sustained_high_performance; do
    run "${g}_${pp}"        "$g" 20 "$pp"
    run "${g}_${pp}_shbuf"  "$g" 20 "$pp" --shared_buffer
  done
done
