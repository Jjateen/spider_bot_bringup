# Health/introspection for the policy controller node.
std_msgs/Header header
float64 rate_hz          # actual control-loop rate
float64 inference_ms     # last ONNX inference time (ms)
float64 action_norm      # L2 norm of the last raw action vector
bool    enabled          # whether the gait is currently armed
