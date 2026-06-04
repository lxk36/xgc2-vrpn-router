#!/usr/bin/env python3
import math
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped


class VrpnRouterProtocolE2ETest(unittest.TestCase):
    def setUp(self):
        self.outputs = {
            "uav1": [],
            "uav2": [],
            "ugv1": [],
        }
        self.diagnostics = []
        self.subscribers = [
            rospy.Subscriber("/uav1/mavros/vision_pose/pose", PoseStamped, self.outputs["uav1"].append),
            rospy.Subscriber("/uav2/mavros/vision_pose/pose", PoseStamped, self.outputs["uav2"].append),
            rospy.Subscriber("/ugv1/vision_pose_custom", PoseStamped, self.outputs["ugv1"].append),
            rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnostics.append),
        ]

    def wait_for_outputs(self, key, min_count, timeout=8.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            if len(self.outputs[key]) >= min_count:
                return True
            rospy.sleep(0.05)
        return False

    def wait_for_diag_problem(self, route_name, problem, timeout=8.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            for array in self.diagnostics:
                for status in array.status:
                    if status.name.endswith(route_name) and problem in status.message:
                        return True
            rospy.sleep(0.05)
        return False

    def saw_diag_problem(self, route_name, problem):
        for array in self.diagnostics:
            for status in array.status:
                if status.name.endswith(route_name) and problem in status.message:
                    return True
        return False

    def latest_diag_value(self, route_name, key):
        for array in reversed(self.diagnostics):
            for status in array.status:
                if not status.name.endswith(route_name):
                    continue
                for value in status.values:
                    if value.key == key:
                        return value.value
        return None

    def assert_pose_near(self, msg, x, y, z, tol=2.0e-5):
        self.assertAlmostEqual(msg.pose.position.x, x, delta=tol)
        self.assertAlmostEqual(msg.pose.position.y, y, delta=tol)
        self.assertAlmostEqual(msg.pose.position.z, z, delta=tol)
        self.assertEqual(msg.header.frame_id, "map")

    def test_vrpn_client_to_router_multi_route_complex_yaml(self):
        self.assertTrue(self.wait_for_outputs("uav1", 10))
        self.assertTrue(self.wait_for_outputs("uav2", 10))
        self.assertTrue(self.wait_for_outputs("ugv1", 5))

        # Values come from the test VRPN server plus per-route field offset and
        # tracker-to-body transform in routes_complex.yaml. The 1e-7 m mocap
        # jitter should be visible over time but never dominate the estimate.
        self.assert_pose_near(self.outputs["uav1"][0], 12.0, 0.0, 0.0)
        self.assert_pose_near(self.outputs["ugv1"][0], 0.0, 0.0, 2.0)

        rospy.sleep(3.5)
        self.assertTrue(self.wait_for_diag_problem("uav1", "input_rate_high"))
        self.assertTrue(self.wait_for_diag_problem("uav2", "vrpn_jump"))
        self.assertFalse(self.saw_diag_problem("uav1", "vrpn_stuck"))

        for key in ("uav1", "uav2", "ugv1"):
            published = self.latest_diag_value(key, "published_count")
            received = self.latest_diag_value(key, "received_count")
            dropped = self.latest_diag_value(key, "dropped_by_rate_count")
            self.assertIsNotNone(published)
            self.assertIsNotNone(received)
            self.assertIsNotNone(dropped)
            self.assertGreater(int(published), 0)
            self.assertGreater(int(received), int(published))
            self.assertGreater(int(dropped), 0)

        self.assert_pose_near(self.outputs["uav2"][-1], 0.0, -3.5, 0.0)
        uav2_after_jump = self.outputs["uav2"][-1].pose.orientation
        yaw = 2.0 * math.atan2(uav2_after_jump.z, uav2_after_jump.w)
        self.assertGreater(abs(yaw), 2.0)


if __name__ == "__main__":
    rospy.init_node("vrpn_router_protocol_e2e_test")
    rostest.rosrun("vrpn_router", "vrpn_router_protocol_e2e", VrpnRouterProtocolE2ETest)
