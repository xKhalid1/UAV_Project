# Result of a completed interception attempt.
std_msgs/Header header
bool detected                # an intruder was detected
bool intercepted             # the interception was executed successfully
string location              # human-readable location label (e.g. "W4")
float64 latitude_deg
float64 longitude_deg
float32 altitude_m
float32 detection_confidence
string approved_by           # "human" | "llm" | "none"
string message
