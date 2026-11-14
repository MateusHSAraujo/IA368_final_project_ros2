import os
import sys

_PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                    "ros2_ws", "src", "vr_robot_pkg")
sys.path.insert(0, os.path.normpath(_PKG))
