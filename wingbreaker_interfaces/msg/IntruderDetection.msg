# Detected intruder (drone) from the onboard camera.
std_msgs/Header header
float64 latitude              # estimated target latitude
float64 longitude             # estimated target longitude
float32 altitude              # estimated target altitude (m)
float32 confidence            # YOLO detection confidence 0..1
string threat_level           # LOW | MEDIUM | HIGH
sensor_msgs/RegionOfInterest roi   # pixel bounding box in the camera image
