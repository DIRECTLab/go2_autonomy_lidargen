#!/usr/bin/env python3
"""Autonomous exploration goal sampler for far_planner.

far_planner is goal-directed (it needs an external /goal_point), so this node
keeps it exploring: it samples goals from OBSERVED traversable terrain --
cells of /terrain_map_ext with low elevation -- which automatically keeps
goals inside the mapped building/free space. A new goal is issued when the
current one is reached (proximity) or times out.

Run (ros2_humble env, cyclonedds):
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python scripts/goal_sampler.py [--min-dist 3] [--max-dist 7] \
        [--reach-dist 1.5] [--goal-timeout 150] [--num-goals 0(=forever)]
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

TRAVERSABLE_ELEV = 0.10  # terrain intensity (est. height above ground) below this = ground
OBSTACLE_ELEV = 0.20
CLEARANCE_M = 0.6  # candidate goals must be at least this far from any obstacle cell


class GoalSampler(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("goal_sampler")
        self.args = args
        self.rng = np.random.default_rng(args.seed)

        self.robot_xy: np.ndarray | None = None
        self.terrain: tuple[np.ndarray, np.ndarray] | None = None  # (xyz, elev)
        self.goal_xy: np.ndarray | None = None
        self.goal_time = 0.0
        self.goals_sent = 0

        self.create_subscription(Odometry, "/state_estimation", self._on_odom, 10)
        self.create_subscription(PointCloud2, "/terrain_map_ext", self._on_terrain, 2)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 5)
        self.create_timer(1.0, self._tick)

    def _on_odom(self, msg: Odometry) -> None:
        self.robot_xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def _on_terrain(self, msg: PointCloud2) -> None:
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
        xyz = raw[:, :12].copy().view(np.float32).reshape(-1, 3)
        elev = raw[:, 16:20].copy().view(np.float32).reshape(-1)
        self.terrain = (xyz, elev)

    def _sample_goal(self) -> np.ndarray | None:
        if self.terrain is None or self.robot_xy is None:
            return None
        xyz, elev = self.terrain
        xy = xyz[:, :2]
        dist = np.linalg.norm(xy - self.robot_xy, axis=1)

        ground = (elev < TRAVERSABLE_ELEV) & (dist >= self.args.min_dist) & (dist <= self.args.max_dist)
        if not ground.any():
            return None
        candidates = xy[ground]

        obstacles = xy[elev > OBSTACLE_ELEV]
        if len(obstacles):
            # keep candidates with clearance from every obstacle cell
            picks = self.rng.permutation(len(candidates))[:200]
            for i in picks:
                c = candidates[i]
                if np.min(np.linalg.norm(obstacles - c, axis=1)) > CLEARANCE_M:
                    return c
            return None
        return candidates[self.rng.integers(len(candidates))]

    def _publish_goal(self, goal_xy: np.ndarray) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x, msg.point.y, msg.point.z = float(goal_xy[0]), float(goal_xy[1]), 0.0
        self.goal_pub.publish(msg)
        self.goal_xy = goal_xy
        self.goal_time = time.time()
        self.goals_sent += 1
        self.get_logger().info(f"goal #{self.goals_sent}: ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})")

    def _tick(self) -> None:
        if self.robot_xy is None or self.terrain is None:
            return

        if self.goal_xy is not None:
            reached = np.linalg.norm(self.robot_xy - self.goal_xy) < self.args.reach_dist
            # NOTE: wall-clock timeout; the sim can run several times slower than
            # realtime in heavy scenes, so keep this generous.
            timed_out = time.time() - self.goal_time > self.args.goal_timeout
            if not (reached or timed_out):
                # re-publish periodically so a late-starting far_planner still gets it
                if int(time.time() - self.goal_time) % 5 == 0:
                    self._republish()
                return
            self.get_logger().info("goal " + ("reached" if reached else "timed out"))
            self.goal_xy = None
            if self.args.num_goals and self.goals_sent >= self.args.num_goals:
                self.get_logger().info("goal budget exhausted, exiting")
                raise SystemExit

        goal = self._sample_goal()
        if goal is not None:
            self._publish_goal(goal)

    def _republish(self) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x, msg.point.y = float(self.goal_xy[0]), float(self.goal_xy[1])
        self.goal_pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--min-dist", type=float, default=3.0)
    parser.add_argument("--max-dist", type=float, default=7.0)
    parser.add_argument("--reach-dist", type=float, default=1.5)
    parser.add_argument("--goal-timeout", type=float, default=150.0)
    parser.add_argument("--num-goals", type=int, default=0, help="stop after N goals (0 = run forever)")
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    rclpy.init()
    node = GoalSampler(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
