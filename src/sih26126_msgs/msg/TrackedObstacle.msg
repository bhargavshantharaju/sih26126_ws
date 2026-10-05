# Copyright 2026 DRISTHI Project
# Licensed under the Apache License, Version 2.0
#
# A single tracked obstacle from the 3D LiDAR obstacle tracker

builtin_interfaces/Time stamp
string id                  # unique track ID (e.g. "obs_001")
geometry_msgs/Point position       # centre in the reference frame
geometry_msgs/Vector3 velocity     # estimated velocity m/s
geometry_msgs/Point predicted_position  # position in 1 second
geometry_msgs/Vector3 dimensions   # bounding box x=width y=depth z=height
float32 confidence         # 0-1
float32 age_sec            # seconds since first seen
float32 last_seen_sec      # seconds since last LiDAR update
bool is_dynamic            # true if velocity > threshold
string classification      # STATIC, DYNAMIC, UNKNOWN
string frame_id            # reference frame (typically base_link or odom)
