#!/usr/bin/env python3
import math
import random
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped


class VrpnRouterE2ETest(unittest.TestCase):
    noise_amplitude_m = 1.0e-7

    def setUp(self):
        self.outputs = []
        self.diagnostics = []
        self.rng = random.Random(7)
        self.pub = rospy.Publisher("/test/vrpn/uav1/pose", PoseStamped, queue_size=100)
        self.ref_pub = rospy.Publisher("/uav1/mavros/local_position/pose", PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber("/uav1/mavros/vision_pose/pose", PoseStamped, self.outputs.append)
        self.diag_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnostics.append)
        rospy.sleep(0.5)

    def pose(self, x, y=0.0, z=0.0):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def noisy_pose(self, x, y=0.0, z=0.0, amplitude=None):
        amplitude = self.noise_amplitude_m if amplitude is None else amplitude
        return self.pose(
            x + self.rng.uniform(-amplitude, amplitude),
            y + self.rng.uniform(-amplitude, amplitude),
            z + self.rng.uniform(-amplitude, amplitude),
        )

    def publish_burst(self, hz, duration, start_x=0.0, velocity=0.0):
        rate = rospy.Rate(hz)
        count = int(hz * duration)
        for i in range(count):
            msg = self.noisy_pose(start_x + velocity * (float(i) / hz))
            self.pub.publish(msg)
            rate.sleep()

    def wait_for_outputs(self, count, timeout=3.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            if len(self.outputs) >= count:
                return True
            rospy.sleep(0.02)
        return False

    def wait_for_diag_problem(self, problem, timeout=3.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            for array in self.diagnostics:
                for status in array.status:
                    if status.name.endswith("uav1") and problem in status.message:
                        return True
            rospy.sleep(0.05)
        return False

    def test_offset_downsample_jump_stuck_timeout_and_reference_delta(self):
        self.publish_burst(120, 1.0, start_x=1.0, velocity=0.05)
        rospy.sleep(0.3)
        self.assertGreater(len(self.outputs), 30)
        self.assertLess(len(self.outputs), 65)
        self.assertTrue(self.wait_for_diag_problem("input_rate_high"))

        first = self.outputs[0]
        self.assertAlmostEqual(first.pose.position.x, 12.0, delta=5e-6)
        self.assertLess(abs(first.pose.position.y), 5e-6)
        self.assertLess(abs(first.pose.position.z), 5e-6)
        self.assertEqual(first.header.frame_id, "map")

        self.pub.publish(self.pose(100.0))
        rospy.sleep(0.2)
        self.assertTrue(self.wait_for_diag_problem("vrpn_jump"))

        self.ref_pub.publish(self.pose(-100.0))
        rospy.sleep(0.2)
        self.assertTrue(self.wait_for_diag_problem("reference_delta_high"))

        for _ in range(30):
            self.pub.publish(self.noisy_pose(2.0))
            rospy.sleep(0.01)
        self.assertTrue(self.wait_for_diag_problem("vrpn_stuck"))

        rospy.sleep(0.4)
        self.assertTrue(self.wait_for_diag_problem("vrpn_timeout"))


if __name__ == "__main__":
    rospy.init_node("vrpn_router_e2e_test")
    rostest.rosrun("vrpn_router", "vrpn_router_e2e", VrpnRouterE2ETest)
