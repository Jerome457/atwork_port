#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json

from atwork_commander_msgs.msg import ObjectTask
from std_msgs.msg import String


class AtworkTaskParser(Node):

    def __init__(self):
        super().__init__("atwork_task_parser")

        # Subscribe to referee task
        self.subscription = self.create_subscription(
            ObjectTask,
            "/atwork_commander/object_task",
            self.task_callback,
            10
        )

        # Publish parsed 2D task array
        self.publisher = self.create_publisher(
            String,
            "/parsed_object_tasks",
            10
        )

        self.get_logger().info("Atwork Task Parser + Publisher Node Started")

    def task_callback(self, msg: ObjectTask):
        """
        Converts ObjectTask message into 2D string array and publishes it.

        Format:
        [
          ["11", "WS01", "WS04"],
          ["13", "WS01", "WS04"],
          ["12", "WS02", "WS04"],
          ["16", "WS04", "WS02"]
        ]
        """

        task_array_2d = []

        for subtask in msg.subtasks:
            object_id = str(subtask.object.object)
            source_ws = str(subtask.source)
            destination_ws = str(subtask.destination)

            task_array_2d.append([object_id, source_ws, destination_ws])

        # Convert to JSON string for ROS transport
        json_msg = String()
        json_msg.data = json.dumps(task_array_2d)

        # Publish
        self.publisher.publish(json_msg)

        # Debug
        self.get_logger().info("Published parsed task table:")
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
