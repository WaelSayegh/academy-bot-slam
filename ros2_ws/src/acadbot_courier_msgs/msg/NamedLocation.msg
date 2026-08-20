# Human-readable location key used in RequestDelivery/ExecuteDelivery
# (e.g. "reception", "lab_bench") — matches a name in the locations YAML.
string name

# 2D pose in the map frame. yaw is used to build the Nav2 goal quaternion.
float64 x
float64 y
float64 yaw