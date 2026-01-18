#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import hashlib

from atwork_commander_msgs.msg import ObjectTask
from std_msgs.msg import String


class AtworkTaskParser(Node):

    def __init__(self):
        super().__init__("atwork_task_parser")

        self.subscription = self.create_subscription(
            ObjectTask,
            "/atwork_commander/object_task",
            self.task_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            "/parsed_object_tasks",
            10
        )

        self.get_logger().info("Atwork Task Parser + Publisher Node Started")

        # Store hash of last published task
        self.last_task_hash = None

    def task_callback(self, msg: ObjectTask):

        task_array_2d = []

        for subtask in msg.subtasks:
            object_id = str(subtask.object.object)
            source_ws = str(subtask.source)
            destination_ws = str(subtask.destination)

            task_array_2d.append([
                object_id,
                source_ws,
                destination_ws,
            ])

        # Convert to canonical JSON string (sorted for stable hashing)
        task_json = json.dumps(task_array_2d, sort_keys=True)

        # Hash it
        current_hash = hashlib.sha256(task_json.encode()).hexdigest()

        # If identical to last message → ignore
        if current_hash == self.last_task_hash:
            self.get_logger().debug("Duplicate task received, ignoring.")
            return

        # New task detected
        self.last_task_hash = current_hash

        json_msg = String()
        json_msg.data = task_json
        self.publisher.publish(json_msg)

        self.get_logger().info("Published NEW parsed task table:")
        for row in task_array_2d:
            self.get_logger().info(f"  {row}")


def main(args=None):
    rclpy.init(args=args)
    node = AtworkTaskParser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
