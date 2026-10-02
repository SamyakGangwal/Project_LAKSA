#!/usr/bin/env python3
"""Print the publisher count of each topic (after a discovery wait), one per line.

    topic_publishers.py /laksa/command /laksa/brake
    -> /laksa/command 1 drive_supervisor
       /laksa/brake 1 drive_supervisor

Exit status 0 on success, 1 on ROS init/discovery failure.  Used by
restart_supervisor.sh and start_reliability.sh so they do not depend on
the ros2 daemon's cache, which can list a node that just exited.
"""

import sys
import time

import rclpy


def main() -> int:
    topics = sys.argv[1:]
    if not topics:
        print(__doc__, file=sys.stderr)
        return 64
    try:
        rclpy.init()
    except Exception as exc:
        print(f"topic_publishers: rclpy.init() failed: {exc}", file=sys.stderr)
        return 1
    try:
        node = rclpy.create_node("laksa_topic_publishers_probe")
    except Exception as exc:
        print(f"topic_publishers: create_node failed: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        for topic in topics:
            infos = node.get_publishers_info_by_topic(topic)
            names = ",".join(sorted(i.node_name for i in infos)) or "-"
            print(f"{topic} {len(infos)} {names}")
    except Exception as exc:
        print(f"topic_publishers: discovery failed: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
