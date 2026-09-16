#!/usr/bin/env python3
"""Contention-robust QNN latency harness for the pi05 VLA context binaries on QCS9075.

Design notes (why it looks like this):
  * This harness times ONE graph at a time via qnn-net-run, serialized. Never run two at once.
    (Note: the board actually exposes TWO HTP devices -- see bench/qnn_device_probe.cpp. This
    harness predates that discovery and always measures the default device 0; the dual-device
    split is measured by ros2_ws/src/qrb_ros_vla/src/pi05_bench.cpp instead.)
  * The box is shared with other workloads, so conditions are INTERLEAVED (round-robin over
    repeats) rather than run in blocks. Any drift in load or temperature then hits every
    condition roughly equally instead of penalising whichever condition ran last.
  * The primary statistic is MIN over inferences, not mean. CPU/DDR contention can only ever
    make a run slower, never faster, so the minimum is the least-biased estimator of the
    achievable latency. Mean and max are recorded too, as a contention indicator.
  * loadavg and the hottest thermal zone are sampled around every run so a contaminated
    sample can be identified after the fact instead of silently averaged in.
  * NPU execution is asserted, not assumed: a run only counts if the backend reported
    accelerator time and a non-zero HVX thread count.

Usage:
  ./measure.py --plan baseline --repeats 3
  ./measure.py --plan knobs --repeats 3 --graphs action_expert
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path

ART = Path("/home/ubuntu/QRB-ROS-VLA/artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075")
HERE = Path(__file__).resolve().parent
WORK = Path("/tmp/pi05_bench")
BACKEND = "/usr/lib/libQnnHtp.so"
GRAPHS = ["vision_encoder", "token_emb", "backbone", "action_expert"]
ENV = {**os.environ, "LD_LIBRARY_PATH": "/usr/lib", "ADSP_LIBRARY_PATH": "/usr/lib/dsp/cdsp"}


# ---------------------------------------------------------------- system state

def loadavg() -> float:
    return float(Path("/proc/loadavg").read_text().split()[0])


def hottest_c() -> float:
    hi = 0.0
    for z in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            hi = max(hi, int(z.read_text().strip()) / 1000.0)
        except (OSError, ValueError):
            pass
    return hi


# ---------------------------------------------------------- config-file emission

# Two DIFFERENT corner enums are in play, and mixing them up is a hard config-load failure:
#   dcvsControl bus/core corners -> DCVS_VOLTAGE_VCORNER_*   ("Invalid Voltage Corner Passed")
#   hmxControl  corners          -> DCVS_EXP_VCORNER_*        ("Invalid Exp Voltage Corner Passed")
MAXV = "DCVS_VOLTAGE_VCORNER_MAX_VOLTAGE_CORNER"
MAXEXP = "DCVS_EXP_VCORNER_MAX"


def max_dcvs(sustain: bool = True, dcvs_off: bool = True, hmx: bool = True) -> dict:
    """The pinned-to-maximum DCVS block, per the schema embedded in
    libQnnHtpNetRunExtensions.so (definitions/dcvsControl + definitions/hmxControl).

    enableDcvs=false stops the DSP clock being scaled down between calls; pinning every
    bus/core/HMX corner to maximum removes the ramp-up a fresh execute would otherwise pay.

    Empirically determined constraints on this SDK (2.46), all verified by bisection:
      * perf_profile MUST be "custom" whenever bus/core corners are given, else
        "Custom configuration of Bus and Core voltage corners requires perfProfile
        specification as CUSTOM".
      * sleep_disable / sleep_latency_us must NOT appear alongside the corners - adding them
        makes the parser stop recognising the corners and it then rejects perf_profile=custom.
      * hmxControl corners must use the DCVS_EXP_VCORNER_* spelling.
    """
    block: dict = {
        "graphState": ["inference_start"],
        "sustainInferencePerfState": sustain,
        "dcvsControl": {
            **({"enableDcvs": False} if dcvs_off else {}),
            "busVoltageCornerMin": MAXV,
            "busVoltageCornerTarget": MAXV,
            "busVoltageCornerMax": MAXV,
            "coreVoltageCornerMin": MAXV,
            "coreVoltageCornerTarget": MAXV,
            "coreVoltageCornerMax": MAXV,
        },
    }
    if hmx:
        block["hmxControl"] = {
            "hmxVoltageCornerMin": MAXEXP,
            "hmxVoltageCornerTarget": MAXEXP,
            "hmxVoltageCornerMax": MAXEXP,
        }
    return block


def write_config(tag: str, graph_names: list[str], cond: dict) -> str | None:
    """Emit the two-file backend-extensions config qnn-net-run expects.

    Outer file points at libQnnHtpNetRunExtensions.so; inner file carries the HTP schema:
      devices[].cores[]  runtime power/RPC knobs (apply to an already-finalized graph)
      graphs[]           compile-time graph knobs (expected inert for --retrieve_context)
      context{}          context-creation knobs

    cond["no_cfg"] suppresses the file entirely, which is the control needed to separate
    "this knob helped" from "loading the backend-extensions library at all helped".
    """
    if cond.get("no_cfg"):
        return None
    core, graph, context = cond.get("core", {}), cond.get("graph", {}), cond.get("context", {})
    dev_extra = cond.get("device", {})
    inner: dict = {}
    if graph:
        inner["graphs"] = [{"graph_names": graph_names, **graph}]
    if not cond.get("no_devices"):
        # Per the schema, core_id is an array on the DEVICE, not a key inside cores[].
        dev = {"device_id": 0, "core_id": [0], "dsp_arch": "v73", **dev_extra}
        dev = {k: v for k, v in dev.items() if v is not None}  # None => omit the key
        dev["cores"] = [dict(core)]
        inner["devices"] = [dev]
    if context:
        inner["context"] = context
    ip = WORK / f"htp_{tag}.json"
    op = WORK / f"cfg_{tag}.json"
    ip.write_text(json.dumps(inner, indent=2))
    op.write_text(json.dumps({
        "backend_extensions": {
            "shared_library_path": "libQnnHtpNetRunExtensions.so",
            "config_file_path": str(ip),
        }
    }, indent=2))
    return str(op)


# ------------------------------------------------------------------- profiling

_PROF_CACHE: dict[str, str] = {}


def profile_text(log: Path) -> str:
    key = str(log)
    if key not in _PROF_CACHE:
        try:
            _PROF_CACHE[key] = subprocess.run(
                ["qnn-profile-viewer", "--input_log", str(log)],
                capture_output=True, text=True, timeout=180, env=ENV).stdout
        except Exception as exc:  # noqa: BLE001 - report, do not crash a long sweep
            _PROF_CACHE[key] = f"PROFILE_VIEWER_FAILED {exc}"
    return _PROF_CACHE[key]


def parse_profile(log: Path) -> dict:
    t = profile_text(log)
    out: dict = {}

    m = re.search(r"Init Stats:\s*-+\s*NetRun:\s+(\d+) us", t)
    out["init_ms"] = int(m.group(1)) / 1000.0 if m else None

    for label, section in (("avg", "Execute Stats (Average):"),
                           ("min", "Execute Stats (Min):"),
                           ("max", "Execute Stats (Max):")):
        part = t.split(section)
        if len(part) < 2:
            continue
        blk = part[1][:2000]
        m = re.search(r"NetRun:\s+(\d+) us", blk)
        if m:
            out[f"netrun_{label}_ms"] = int(m.group(1)) / 1000.0
        m = re.search(r"Accelerator \(execute\) time\):\s+(\d+) us", blk)
        if m:
            out[f"accel_{label}_ms"] = int(m.group(1)) / 1000.0
        m = re.search(r"HVX threads used\):\s+(\d+)", blk)
        if m:
            out["hvx_threads"] = int(m.group(1))

    m = re.search(r"IPS \(includes IO and misc\. time\):\s+([\d.]+)", t)
    out["ips"] = float(m.group(1)) if m else None
    return out


def npu_ok(p: dict) -> bool:
    """Guard against a silent CPU fallback being reported as a win."""
    return bool(p.get("accel_avg_ms")) and bool(p.get("hvx_threads"))


# ----------------------------------------------------------------------- runner

def run_once(cond: dict, graph: str, n: int) -> dict:
    tag = f"{graph}__{cond['tag']}"
    out = WORK / f"out_{tag}"
    shutil.rmtree(out, ignore_errors=True)

    cfg = write_config(tag, [f"pi05_{graph}"], cond)

    cmd = ["qnn-net-run", "--backend", BACKEND,
           "--retrieve_context", str(ART / f"{graph}.bin"),
           "--input_list", str(HERE / f"{graph}.inputlist"),
           "--output_dir", str(out), "--num_inferences", str(n),
           "--keep_num_outputs", "1", "--profiling_level", "basic"]
    if cond.get("perf"):
        cmd += ["--perf_profile", cond["perf"]]
    if cfg:
        cmd += ["--config_file", cfg]
    cmd += cond.get("extra", [])

    l0, t0 = loadavg(), hottest_c()
    w0 = time.perf_counter()
    proc = subprocess.run(cmd, capture_output=True, text=True, env=ENV, timeout=1800)
    wall = time.perf_counter() - w0
    l1, t1 = loadavg(), hottest_c()

    log = out / "qnn-profiling-data_0.log"
    prof = parse_profile(log) if log.exists() else {}
    stderr_tail = "\n".join(
        ln for ln in (proc.stdout + proc.stderr).splitlines()
        if re.search(r"error|fail|unsupported|ignor|invalid|warn", ln, re.I)
        and "QNN_LOG_LEVEL" not in ln)[:800]

    return {
        "graph": graph, "cond": cond["tag"], "desc": cond.get("desc", ""),
        "n": n, "rc": proc.returncode, "wall_s": round(wall, 3),
        "load_before": l0, "load_after": l1,
        "temp_before_c": t0, "temp_after_c": t1,
        "npu_confirmed": npu_ok(prof),
        **prof,
        "diag": stderr_tail,
        "cmd": " ".join(cmd),
    }


# ------------------------------------------------------------------------ plans

def plan_baseline() -> list[dict]:
    """perf_profile x shared_buffer, using only CLI flags - no config file at all."""
    conds = []
    for pp in ["default", "balanced", "high_performance", "sustained_high_performance", "burst"]:
        conds.append({"tag": f"pp_{pp}", "perf": pp, "no_cfg": True,
                      "desc": f"perf_profile={pp}, no config file"})
        conds.append({"tag": f"pp_{pp}_shbuf", "perf": pp, "no_cfg": True,
                      "extra": ["--shared_buffer"],
                      "desc": f"perf_profile={pp} + shared_buffer, no config file"})
    return conds


def plan_isolate() -> list[dict]:
    """Why does merely PASSING a config file speed things up?

    The knobs plan showed every config-file condition landing at the same faster number,
    including options that should be inert on a finalized graph. So the speedup is not
    attributable to any individual knob. These conditions strip the config down until the
    responsible element is identified, holding perf_profile and shared_buffer fixed.
    """
    base = {"perf": "burst", "extra": ["--shared_buffer"]}
    return [
        {"tag": "z_nocfg", **base, "no_cfg": True,
         "desc": "CONTROL: no config file (CLI flags only)"},
        {"tag": "z_cfg_bare", **base, "no_devices": True,
         "desc": "config file present but EMPTY object"},
        {"tag": "z_cfg_dev", **base,
         "desc": "config file, devices block only (device_id/core_id/dsp_arch), no knobs"},
        {"tag": "z_cfg_nodsparch", **base, "device": {"dsp_arch": None},
         "desc": "devices block without dsp_arch"},
        {"tag": "z_cfg_graphonly", **base, "no_devices": True, "graph": {"O": 3},
         "desc": "graphs block only, no devices block"},
        {"tag": "z_cfg_pp_in_core", **base, "core": {"perf_profile": "burst"},
         "desc": "devices block + perf_profile=burst inside cores[]"},
        # Does the effect survive without the CLI perf flag, and without shared_buffer?
        {"tag": "z_cfg_dev_noshbuf", "perf": "burst",
         "desc": "devices block only, WITHOUT shared_buffer"},
        {"tag": "z_nocfg_noshbuf", "perf": "burst", "no_cfg": True,
         "desc": "CONTROL: no config file, WITHOUT shared_buffer"},
    ]


def plan_knobs() -> list[dict]:
    """Runtime power/RPC knobs, layered on burst + shared_buffer + a devices block.

    rpc_polling_time makes the DSP spin-wait instead of sleeping on an interrupt, removing
    wake-up latency from every execute call - most valuable for a short graph invoked in a
    tight loop, i.e. the action expert's denoise steps.
    """
    base = {"perf": "burst", "extra": ["--shared_buffer"]}
    conds = [
        {"tag": "k_base", **base, "no_cfg": True, "desc": "CONTROL: no config file"},
        {"tag": "k_dev", **base, "desc": "devices block, no knobs (config-file reference)"},
    ]
    for us in (100, 1000, 9999):
        conds.append({"tag": f"k_poll{us}", **base, "core": {"rpc_polling_time": us},
                      "desc": f"rpc_polling_time={us}us"})
    for us in (0, 40, 100):
        conds.append({"tag": f"k_ctrl{us}", **base, "core": {"rpc_control_latency": us},
                      "desc": f"rpc_control_latency={us}us"})
    conds += [
        {"tag": "k_poll_ctrl", **base,
         "core": {"rpc_polling_time": 9999, "rpc_control_latency": 0},
         "desc": "rpc_polling_time=9999 + rpc_control_latency=0"},
        {"tag": "k_hmx", **base, "core": {"hmx_timeout_us": 10000},
         "desc": "hmx_timeout_us=10000"},
        {"tag": "k_adapt", **base, "core": {"adaptive_polling_time": 1000},
         "desc": "adaptive_polling_time=1000us"},
        # ddr_perf_mode is a BOOLEAN in the schema, and is only honoured when the bus
        # voltage corners are all pinned to MAX - hence it appears in the max plan too.
        {"tag": "k_ddr", **base, "core": {"ddr_perf_mode": True},
         "desc": "ddr_perf_mode=true (bus corners not pinned; expect a warning)"},
        # Graph-level options: expected INERT on a finalized context binary. Measured to prove it.
        {"tag": "g_hvx6", **base, "graph": {"hvx_threads": 6},
         "desc": "hvx_threads=6 (graph-level; expected inert)"},
        {"tag": "g_vtcm4", **base, "graph": {"vtcm_mb": 4},
         "desc": "vtcm_mb=4 (graph-level; expected inert)"},
        {"tag": "g_fp16relax", **base, "graph": {"fp16_relaxed_precision": 1},
         "desc": "fp16_relaxed_precision=1 (graph-level; expected inert)"},
        {"tag": "g_dlbc", **base, "graph": {"dlbc": 1, "dlbc_weights": 1},
         "desc": "dlbc=1 (compiled binary has htpDlbc=0; expected inert)"},
        {"tag": "g_shareio", **base, "graph": {"share_io_buffer": True},
         "desc": "share_io_buffer=true (graph-level; expected inert)"},
    ]
    return conds


def plan_max() -> list[dict]:
    """Pin every clock domain to maximum via perf_profile=custom + explicit voltage corners."""
    base = {"perf": "burst", "extra": ["--shared_buffer"]}
    return [
        {"tag": "m_base", **base, "desc": "reference: burst + shared_buffer + devices block"},
        {"tag": "m_custom", **base,
         "core": {"perf_profile": "custom", "custom": [max_dcvs()]},
         "desc": "perf_profile=custom, all voltage corners pinned MAX, DCVS off"},
        {"tag": "m_custom_poll", **base,
         "core": {"perf_profile": "custom", "rpc_polling_time": 9999,
                  "rpc_control_latency": 0, "custom": [max_dcvs()]},
         "desc": "custom MAX corners + rpc_polling_time=9999 + rpc_control_latency=0"},
        {"tag": "m_custom_ddr", **base,
         "core": {"perf_profile": "custom", "ddr_perf_mode": True,
                  "rpc_polling_time": 9999, "rpc_control_latency": 0,
                  "custom": [max_dcvs()]},
         "desc": "custom MAX corners + ddr_perf_mode + polling (everything at once)"},
        {"tag": "m_custom_nosustain", **base,
         "core": {"perf_profile": "custom", "custom": [max_dcvs(sustain=False)]},
         "desc": "custom MAX corners, sustainInferencePerfState=false"},
        {"tag": "m_custom_nohmx", **base,
         "core": {"perf_profile": "custom", "custom": [max_dcvs(hmx=False)]},
         "desc": "custom MAX corners, no hmxControl (isolates the HMX pin)"},
        {"tag": "m_custom_dcvson", **base,
         "core": {"perf_profile": "custom", "custom": [max_dcvs(dcvs_off=False)]},
         "desc": "custom MAX corners but DCVS left enabled"},
        {"tag": "m_all", **base,
         "core": {"perf_profile": "custom", "ddr_perf_mode": True,
                  "rpc_polling_time": 9999, "rpc_control_latency": 0,
                  "hmx_timeout_us": 10000, "custom": [max_dcvs()]},
         "extra": ["--shared_buffer"],
         "desc": "EVERYTHING: MAX corners + DCVS off + ddr_perf_mode + polling + hmx timeout"},
    ]


def plan_init() -> list[dict]:
    """Attack context-creation time (~1.04 s summed over the four binaries)."""
    base = {"perf": "burst", "extra": ["--shared_buffer"]}
    return [
        {"tag": "i_base", **base, "desc": "init reference (devices block only)"},
        {"tag": "i_skipval", **base, "context": {"skip_validation_on_binary_section": True},
         "desc": "skip_validation_on_binary_section"},
        {"tag": "i_accel", **base, "context": {"init_acceleration": True},
         "desc": "init_acceleration"},
        {"tag": "i_budget", **base, "context": {"file_read_memory_budget_in_mb": 2048},
         "desc": "file_read_memory_budget_in_mb=2048"},
        {"tag": "i_ioest", **base, "context": {"io_memory_estimation": True},
         "desc": "io_memory_estimation"},
        {"tag": "i_udma", **base, "context": {"extended_udma": True},
         "desc": "extended_udma"},
        {"tag": "i_detach", **base, "context": {"detachable_buffers_enabled": True},
         "desc": "detachable_buffers_enabled"},
        {"tag": "i_all", **base,
         "context": {"skip_validation_on_binary_section": True, "init_acceleration": True,
                     "file_read_memory_budget_in_mb": 2048, "io_memory_estimation": True},
         "desc": "skip_validation + init_acceleration + read budget + io estimation"},
    ]


def plan_profiles() -> list[dict]:
    """Falsification test for "the extensions library is what applies the perf profile".

    If that mechanism is right, perf_profile set INSIDE cores[] must differentiate - and
    power_saver in particular must be measurably SLOWER than burst. If every profile lands on
    the same number, the config-file speedup is caused by something else and the explanation
    is wrong.
    """
    conds = [{"tag": "p_nocfg_burst", "perf": "burst", "no_cfg": True,
              "extra": ["--shared_buffer"], "desc": "CONTROL: no config file, CLI burst"},
             {"tag": "p_nocfg_saver", "perf": "power_saver", "no_cfg": True,
              "extra": ["--shared_buffer"],
              "desc": "CONTROL: no config file, CLI power_saver (expect same as CLI burst)"}]
    for pp in ["power_saver", "low_balanced", "balanced", "high_performance",
               "sustained_high_performance", "burst"]:
        conds.append({"tag": f"p_cfg_{pp}", "perf": "burst", "extra": ["--shared_buffer"],
                      "core": {"perf_profile": pp},
                      "desc": f"config file, cores[].perf_profile={pp}"})
    return conds


def plan_cliprof() -> list[dict]:
    """The clean version of the perf-profile test.

    The earlier plan_profiles run was confounded: it varied cores[].perf_profile while holding
    the CLI --perf_profile at burst, and the CLI value wins (verbose logging shows
    "Setting Perf Profile to 5" in every arm). Here the CLI flag is the variable and the
    config file is a bare {} whose only job is to get the extensions library loaded.

    Verbose logs confirm the CLI flag reaches the backend as distinct enums:
    power_saver=18, low_balanced=0, balanced=1, high_performance=3,
    sustained_high_performance=4, burst=5.
    """
    conds = []
    for pp in ["power_saver", "low_balanced", "balanced", "high_performance",
               "sustained_high_performance", "burst"]:
        conds.append({"tag": f"c_{pp}", "perf": pp, "extra": ["--shared_buffer"],
                      "no_devices": True, "desc": f"bare config file + CLI --perf_profile={pp}"})
    conds.append({"tag": "c_nocfg_burst", "perf": "burst", "no_cfg": True,
                  "extra": ["--shared_buffer"], "desc": "CONTROL: no config file, CLI burst"})
    return conds


def plan_cli() -> list[dict]:
    """Remaining qnn-net-run flags that could move latency or init cost.

    --use_mmap is the interesting one: these four binaries total ~3 GB, and context creation
    was measured at 124-570 ms per graph, so how the blob reaches memory is a real cost.
    """
    base = {"perf": "burst", "no_devices": True}
    return [
        {"tag": "f_ref", **base, "extra": ["--shared_buffer"],
         "desc": "reference: bare config + burst + shared_buffer"},
        {"tag": "f_mmap", **base, "extra": ["--shared_buffer", "--use_mmap"],
         "desc": "+ --use_mmap (mmap the context binary instead of reading it)"},
        {"tag": "f_mmap_only", **base, "extra": ["--use_mmap"],
         "desc": "--use_mmap without shared_buffer"},
        {"tag": "f_async", **base, "extra": ["--shared_buffer", "--asynchronous"],
         "desc": "+ --asynchronous graph execution"},
        {"tag": "f_cache4", **base,
         "extra": ["--shared_buffer", "--max_input_cache_tensor_sets", "4"],
         "desc": "+ --max_input_cache_tensor_sets=4 (input tensor set caching)"},
        {"tag": "f_native", **base,
         "extra": ["--shared_buffer", "--use_native_input_files", "--use_native_output_files"],
         "desc": "+ native input/output files (skip float parse/convert on the host)"},
    ]


PLANS = {"baseline": plan_baseline, "isolate": plan_isolate, "knobs": plan_knobs,
         "max": plan_max, "profiles": plan_profiles, "cliprof": plan_cliprof,
         "init": plan_init, "cli": plan_cli}


# ------------------------------------------------------------------------- main

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", choices=sorted(PLANS), default="baseline")
    ap.add_argument("--graphs", nargs="*", default=GRAPHS)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--inferences", type=int, default=20)
    ap.add_argument("--out", default=None)
    ap.add_argument("--max-load", type=float, default=None,
                    help="if set, wait for 1-min loadavg below this before each run")
    args = ap.parse_args()

    WORK.mkdir(parents=True, exist_ok=True)
    outp = Path(args.out or HERE / f"results_{args.plan}.jsonl")
    conds = PLANS[args.plan]()

    # Interleave: repeat-major, and rotate condition order per repeat so no condition is
    # systematically first (cold) or last (hot / most contended).
    jobs = []
    for r in range(args.repeats):
        for g in args.graphs:
            rot = conds[r % len(conds):] + conds[:r % len(conds)]
            jobs += [(r, g, c) for c in rot]

    print(f"plan={args.plan} graphs={args.graphs} conds={len(conds)} "
          f"repeats={args.repeats} runs={len(jobs)} -> {outp}", flush=True)

    with outp.open("w") as fh:
        for i, (r, g, c) in enumerate(jobs, 1):
            if args.max_load is not None:
                for _ in range(60):
                    if loadavg() < args.max_load:
                        break
                    time.sleep(5)
            rec = run_once(c, g, args.inferences)
            rec["repeat"] = r
            fh.write(json.dumps(rec) + "\n")
            fh.flush()
            flag = "" if rec["npu_confirmed"] else "  <-- NPU NOT CONFIRMED"
            print(f"[{i}/{len(jobs)}] r{r} {g:15s} {c['tag']:16s} "
                  f"min={rec.get('netrun_min_ms')} avg={rec.get('netrun_avg_ms')} "
                  f"init={rec.get('init_ms')} load={rec['load_before']}{flag}", flush=True)

    summarize(outp)
    return 0


def summarize(path: Path) -> None:
    rows = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
    print(f"\n=== summary: {path.name} (primary statistic = min over inferences, "
          f"best over repeats) ===")
    by: dict[tuple[str, str], list[dict]] = {}
    for r in rows:
        by.setdefault((r["graph"], r["cond"]), []).append(r)

    for graph in dict.fromkeys(r["graph"] for r in rows):
        ks = [k for k in by if k[0] == graph]
        scored = []
        for k in ks:
            mins = [x["netrun_min_ms"] for x in by[k] if x.get("netrun_min_ms")]
            inits = [x["init_ms"] for x in by[k] if x.get("init_ms")]
            if not mins:
                continue
            scored.append((min(mins), statistics.median(mins), min(inits) if inits else None,
                           k[1], by[k][0].get("desc", ""), all(x["npu_confirmed"] for x in by[k])))
        if not scored:
            continue
        scored.sort()
        ref = next((s for s in scored if s[3] in
                    ("pp_default", "z_nocfg", "k_base", "m_base", "i_base")), scored[-1])
        print(f"\n-- {graph} --   (reference: {ref[3]} = {ref[0]:.2f} ms)")
        print(f"   {'cond':18}{'best_min':>10}{'med_min':>10}{'init':>9}{'vs_ref':>9}  npu  desc")
        for best, med, init, tag, desc, ok in scored:
            spd = ref[0] / best if best else 0
            print(f"   {tag:18}{best:10.2f}{med:10.2f}"
                  f"{(init or 0):9.1f}{spd:8.2f}x  {'ok ' if ok else 'FAIL'}  {desc}")


if __name__ == "__main__":
    sys.exit(main())
