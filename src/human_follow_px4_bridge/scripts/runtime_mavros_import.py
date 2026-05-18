#!/usr/bin/env python3
import glob
import os
import sys


def ensure_mavros_python_path():
    candidates = []
    env_prefix = os.environ.get("MAVROS_OVERLAY_PREFIX", "").strip()
    if env_prefix:
        candidates.append(env_prefix)
    candidates.append("/home/coco/.local/ros_noetic_overlay/opt/ros/noetic")

    for prefix in candidates:
        if not prefix:
            continue
        for dist_path in glob.glob(os.path.join(prefix, "lib", "python3*", "dist-packages")):
            if os.path.isdir(os.path.join(dist_path, "mavros_msgs")) and dist_path not in sys.path:
                sys.path.insert(0, dist_path)
