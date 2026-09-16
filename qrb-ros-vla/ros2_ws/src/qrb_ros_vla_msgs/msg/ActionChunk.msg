# One Pi0.5 action chunk: chunk_size timesteps of action_dim values each.
#
# Pi0.5 emits a fixed max_action_dim (32) per timestep and the embodiment's real
# degrees of freedom are the first `action_dof` columns; the rest is padding and
# must be ignored. actions is row-major [chunk_size][action_dim].

std_msgs/Header header

# Language instruction this chunk was conditioned on (already state-augmented).
string task

uint32 chunk_size
uint32 action_dim
uint32 action_dof

# chunk_size * action_dim values, row-major. Un-normalized only if
# `unnormalized` is true; otherwise these are in the policy's normalized space.
float32[] actions
bool unnormalized

# Number of flow-matching denoise steps used to produce this chunk.
uint32 denoise_steps
