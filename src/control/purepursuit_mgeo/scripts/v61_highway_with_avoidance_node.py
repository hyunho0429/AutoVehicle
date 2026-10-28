#!/usr/bin/env python3
"""Keep the uploaded highway implementation; defer activation during avoidance."""
import importlib.util
import os
import rospkg
import rospy
from std_msgs.msg import Bool

source = os.path.join(rospkg.RosPack().get_path("purepursuit_mgeo"),
                      "scripts", "uploaded_highway_lane_strategy_node.py")
spec = importlib.util.spec_from_file_location("v61_uploaded_highway", source)
module = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class HighwayWithAvoidance(module.HighwayLaneStrategyNode):
    def __init__(self):
        self.avoidance_active = True
        self.avoidance_at = None
        super().__init__()
        rospy.Subscriber("/integration/avoidance/active", Bool, self._avoidance_cb, queue_size=1)

    def _avoidance_cb(self, msg):
        self.avoidance_active = bool(msg.data)
        self.avoidance_at = rospy.get_time()

    def _activation_present(self):
        if (self.avoidance_at is None or not 0 <= rospy.get_time()-self.avoidance_at <= 0.7
                or self.avoidance_active):
            return False
        return super()._activation_present()


if __name__ == "__main__":
    HighwayWithAvoidance()
    rospy.spin()
