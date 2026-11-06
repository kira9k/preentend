#!/usr/bin/env python3
"""Add the static car from the Gazebo world to MoveIt's planning scene."""

import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def main():
    rclpy.init()
    node = rclpy.create_node("add_car_collision")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for MoveIt planning scene service")

    if not rclpy.ok():
        node.destroy_node()
        return 1

    car = CollisionObject()
    car.header.frame_id = "world"
    car.id = "car"
    car.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [4.7, 2.05, 1.56]
    car.primitives.append(box)

    pose = Pose()
    pose.position.z = 0.78
    pose.orientation.w = 1.0
    car.primitive_poses.append(pose)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(car)

    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    success = future.result() is not None and future.result().success
    if success:
        node.get_logger().info("Added car collision box to MoveIt planning scene")
    else:
        node.get_logger().error("Could not add car collision box to MoveIt planning scene")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
