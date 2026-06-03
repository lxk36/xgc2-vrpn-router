#!/usr/bin/env python3
import argparse
import math

import rospy
from geometry_msgs.msg import PoseStamped


def make_pose(x, y, z, frame_id):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg


def run(args):
    pub = rospy.Publisher(args.topic, PoseStamped, queue_size=200)
    rospy.sleep(0.5)

    if args.mode == "dropout":
        rate = rospy.Rate(args.rate)
        for _ in range(int(args.rate * args.duration * 0.4)):
            pub.publish(make_pose(0.0, 0.0, 0.0, args.frame_id))
            rate.sleep()
        rospy.sleep(args.duration * 0.6)
        return

    rate = rospy.Rate(args.rate)
    count = int(args.rate * args.duration)
    for i in range(count):
        t = float(i) / max(args.rate, 1.0)
        x = args.velocity * t
        if args.mode == "jump" and i == count // 2:
            x += args.jump
        elif args.mode == "stuck":
            x = 0.0
        elif args.mode == "jitter":
            x += args.jitter * math.sin(i * 1.7)
        pub.publish(make_pose(x, 0.0, 0.0, args.frame_id))
        rate.sleep()


def main():
    parser = argparse.ArgumentParser(description="Publish synthetic VRPN pose streams for vrpn_router e2e testing.")
    parser.add_argument("--topic", default="/test/vrpn/uav1/pose")
    parser.add_argument("--mode", choices=["normal", "jump", "stuck", "dropout", "jitter"], default="normal")
    parser.add_argument("--rate", type=float, default=120.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--velocity", type=float, default=0.1)
    parser.add_argument("--jump", type=float, default=5.0)
    parser.add_argument("--jitter", type=float, default=0.2)
    parser.add_argument("--frame-id", default="map")
    args = parser.parse_args()

    rospy.init_node("vrpn_router_extreme_sim")
    run(args)


if __name__ == "__main__":
    main()
