# Per-chunk latency breakdown, published alongside every ActionChunk so that any
# performance claim about this pipeline is traceable to a live measurement
# instead of an estimate.

std_msgs/Header header

# Wall-clock milliseconds spent inside qrb_inference_manager per stage.
# vision_encoder_ms covers all camera passes; action_expert_ms covers all
# denoise steps. host_packing_ms is CPU-side buffer assembly.
float64 vision_encoder_ms
float64 token_emb_ms
float64 backbone_ms
float64 action_expert_ms
float64 host_packing_ms
float64 total_ms

uint32 camera_passes
uint32 denoise_steps

# Backend library actually loaded, e.g. libQnnHtp.so. Proof-of-NPU lives here:
# a CPU fallback shows up as a different backend and a ~10x worse total_ms.
string backend

# Chunks per second implied by total_ms.
float64 chunks_per_second
