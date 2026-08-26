# Telemetry exposed by the MAVSDK vehicle gateway.
std_msgs/Header header
bool connected
bool armed
bool airborne
float64 latitude_deg
float64 longitude_deg
float32 relative_altitude_m
float32 ground_speed_m_s
float32 heading_deg
float32 battery_percent      # -1.0 when unknown
int32 num_satellites         # -1 when unknown
uint8 gps_fix_type           # 0 when unknown
