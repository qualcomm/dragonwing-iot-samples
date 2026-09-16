#!/usr/bin/env bash
set -uo pipefail
export LD_LIBRARY_PATH=/usr/lib ADSP_LIBRARY_PATH=/usr/lib/dsp/cdsp
ART=/home/ubuntu/QRB-ROS-VLA/artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075
G=$1; N=${2:-20}; PP=${3:-default}; shift 3 || true
OUT=/tmp/out_${G}_${PP}; rm -rf "$OUT"
/usr/bin/time -f "WALL %e s  MAXRSS %M KB" qnn-net-run \
  --backend /usr/lib/libQnnHtp.so --retrieve_context "$ART/$G.bin" \
  --input_list "$G.inputlist" --output_dir "$OUT" \
  --num_inferences "$N" --perf_profile "$PP" --keep_num_outputs 1 \
  --profiling_level basic "$@" 2>&1 | grep -viE "rpcmem|Shell cwd|^qnn-net-run (pid|build|log)"
python3 - "$OUT" <<'PY'
import sys,os,glob,re
d=sys.argv[1]
for f in glob.glob(d+'/**/*.log',recursive=True)+glob.glob(d+'/*.csv'): print('LOG',f)
PY
ls "$OUT" 2>/dev/null | head
