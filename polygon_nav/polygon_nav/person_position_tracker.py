#!/usr/bin/env python3
# person_position_tracker.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

class PersonPositionTracker(Node):
    def __init__(self):
        super().__init__('person_position_tracker')
        
        # Subscriber
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            1  # Queue size 1
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1  # Queue size 1
        )
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            1  # Queue size 1
        )
        
        # Publisher für Person Positionen
        self.position_pub = self.create_publisher(
            PointStamped,
            '/person_position',
            1  # Queue size 1
        )
        
        self.marker_pub = self.create_publisher(
            PoseStamped,
            '/person_marker',
            1  # Queue size 1
        )
        
        # Variablen
        self.current_pose = None
        self.laser_data = None
        self.camera_fov = 1.361  # 78° Field of View (typisch für Kameras)
        
        self.get_logger().info('Person Position Tracker gestartet (Odom-only)')

    def odom_callback(self, msg):
        """Speichert die aktuelle Robot Position"""
        self.current_pose = msg.pose.pose

    def laser_callback(self, msg):
        """Speichert die Laser Daten"""
        self.laser_data = msg

    def detection_callback(self, msg):
        """Verarbeitet Person Detections und berechnet Position"""
        if self.current_pose is None or self.laser_data is None:
            self.get_logger().warn('No odom or laser data yet')
            return
        
        for detection in msg.detections:
            if detection.results[0].hypothesis.class_id == "person":
                confidence = detection.results[0].hypothesis.score
                
                if confidence > 0.5:  # Nur high-confidence Detections
                    person_position = self.calculate_person_position(detection)
                    
                    if person_position:
                        self.publish_person_position(person_position)
                        self.get_logger().info(
                            f'Person at: x={person_position.x:.2f}, y={person_position.y:.2f}'
                        )

    def calculate_person_position(self, detection):
        """Berechnet die Welt-Position der Person"""
        try:
            # 1. Bounding Box Center im Kamerabild
            bbox_center_x = detection.bbox.center.position.x
            bbox_center_y = detection.bbox.center.position.y
            
            # 2. Winkel zur Person berechnen (relativ zur Kamera)
            image_width = 320  # Typische Kamerauflösung anpassen!
            pixel_angle = (bbox_center_x - image_width/2) / image_width * self.camera_fov
            
            # 3. Laser-Distanz in diesem Winkel finden
            if self.laser_data:
                distance = self.get_laser_distance(pixel_angle)
                
                if distance is not None and distance < 10.0:  # Max 10m Distanz
                    # 4. Relative Position zur Kamera berechnen
                    rel_x = distance * math.cos(pixel_angle)
                    rel_y = distance * math.sin(pixel_angle)
                    
                    #self.get_logger().info(
                           # f'Relative Distanz von Bot zu detektierter Person: x={rel_x:.2f}, y={rel_y:.2f} '
                       # )
                    
                    # 5. In Welt-Koordinaten transformieren (MIT ODOM)
                    world_point = self.transform_with_odom(rel_x, rel_y)
                    return world_point
            
        except Exception as e:
            self.get_logger().error(f'Position calculation error: {e}')
        
        return None

    def get_laser_distance(self, angle):
        if self.laser_data is None:
            return None
        
        min_angle = self.laser_data.angle_min    # z.B. -3.14 rad (-180°)
        max_angle = self.laser_data.angle_max    # z.B. +3.14 rad (+180°)
        angle_increment = self.laser_data.angle_increment
        
        # 1. Winkel normalisieren (NUR bei 360° Laser!)
        # Stellen Sie sicher dass der Winkel zwischen -π und +π liegt
        while angle > max_angle:
            angle -= 2 * math.pi
        while angle < min_angle:
            angle += 2 * math.pi
        
        # 2. Index berechnen
        index = int((angle - min_angle) / angle_increment)
        
        # 3. Sicherstellen dass Index gültig ist
        if 0 <= index < len(self.laser_data.ranges):
            distance = self.laser_data.ranges[index]
            if not math.isinf(distance) and distance > self.laser_data.range_min:
                return distance
        
        return None

    def transform_with_odom(self, rel_x, rel_y):
        """Transformiert relative Koordinaten using ODOM data"""
        try:
            # Robot Position und Orientation aus Odom
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            
            # Robot Orientation (Yaw Winkel aus Quaternion)
            orientation = self.current_pose.orientation
            sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            robot_yaw = math.atan2(sin_yaw, cos_yaw)
            self.get_logger().info(f'robot_yaw={robot_yaw:.2f}')
            
            # Relative Position rotieren (basierend auf Robot Orientation)
            rotated_x = rel_x * math.cos(robot_yaw) - rel_y * math.sin(robot_yaw)
            rotated_y = rel_x * math.sin(robot_yaw) + rel_y * math.cos(robot_yaw)
            
            # Absolute Welt-Koordinaten
            world_point = Point()
            world_point.x = robot_x + rotated_x
            world_point.y = robot_y + rotated_y
            world_point.z = 0.0
            
            self.get_logger().info(f'robot_y={robot_y:.2f}')
            self.get_logger().info(f'rotated_y={rotated_y:.2f}')
            
            self.get_logger().info(
                            f'Absolute Verortung von Person im Raum von Bot  absolute_x={world_point.x:.2f}, absolute_y={world_point.y:.2f} '
                        )
            
            return world_point
            
        except Exception as e:
            self.get_logger().warn(f'Odom transformation failed: {e}')
            return None

    def publish_person_position(self, position):
        """Publiziert die Person Position"""
        # Als PointStamped
        point_msg = PointStamped()
        point_msg.header.frame_id = 'odom'  # Jetzt odom statt map!
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point = position
        self.position_pub.publish(point_msg)
        
        # Als PoseStamped für RViz Marker
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'odom'  # Jetzt odom statt map!
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position = position
        pose_msg.pose.orientation.w = 1.0  # Neutral orientation
        self.marker_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonPositionTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
