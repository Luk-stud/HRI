#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from shapely.geometry import Point, Polygon
import numpy as np

class PolygonNavigationClient(Node):
    def __init__(self):
        super().__init__('polygon_navigation_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Polygon Navigation Client initialized")

    def follow_path(self, path_points):
        self.get_logger().info("Waiting for action server...")
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("...still waiting for action server")

        for i, point in enumerate(path_points):
            self.get_logger().info(f"Navigating to point {i+1}/{len(path_points)}")

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = point

            send_goal_future = self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().error(f"Goal {i+1} was rejected!")
                continue  # Versuche den nächsten Punkt

            self.get_logger().info(f"Goal {i+1} accepted, navigating...")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"Point {i+1} reached successfully!")
            else:
                self.get_logger().error(f"Failed to reach point {i+1}. Status: {result.status}")

        self.get_logger().info("All points processed.")
        return True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation progress: {feedback}')

def create_pose_stamped(x, y, z=0.0, orientation_w=1.0, frame_id='map'):
    from rclpy.time import Time
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = Time().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = orientation_w
    return pose

def generate_coverage_path(polygon_points, resolution=0.1):
    """Gibt eine Liste von PoseStamped-Zielen zurück, um das Polygon flächig abzudecken."""
    # Extrahiere Koordinaten für Shapely-Polygon
    coords = [(p.pose.position.x, p.pose.position.y) for p in polygon_points]
    poly = Polygon(coords)

    # Bounding Box berechnen
    min_x, min_y, max_x, max_y = poly.bounds

    # Grid generieren
    x_vals = np.arange(min_x, max_x, resolution)
    y_vals = np.arange(min_y, max_y, resolution)

    points_in_poly = []

    # Zeilenweise abfahren (alternierend)
    for i, y in enumerate(y_vals):
        row = []
        for x in x_vals:
            point = Point(x, y)
            if poly.contains(point):
                row.append(create_pose_stamped(x, y))
        if i % 2 == 1:
            row.reverse()  # Für boustrophedonischen Pfad
        points_in_poly.extend(row)

    return points_in_poly

def main(args=None):
    rclpy.init(args=args)

    # Polygon definieren – geschlossene Form, erster und letzter Punkt sollten gleich sein
    polygon_points = [
        create_pose_stamped(2.43, 0.68),
        create_pose_stamped(2.44, 3.42),
        create_pose_stamped(1.42, 3.37),
        create_pose_stamped(0.90, 1.05),
        create_pose_stamped(2.43, 0.68)  # zurück zum Startpunkt
    ]

    node = PolygonNavigationClient()

    try:
        # Coverage Path berechnen
        coverage_path = generate_coverage_path(polygon_points, resolution=0.2)
        node.get_logger().info(f"Generated {len(coverage_path)} coverage points.")

        # Pfad abfahren
        success = node.follow_path(coverage_path)
        if success:
            node.get_logger().info("Polygon area coverage completed!")
        else:
            node.get_logger().error("Coverage navigation failed!")
    except Exception as e:
        node.get_logger().error(f"Error during navigation: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

