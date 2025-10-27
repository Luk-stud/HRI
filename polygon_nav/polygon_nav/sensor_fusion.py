#!/usr/bin/env python3
# sensor_fusion.py
# Fuses data from YOLO, Hand Gesture Detector and other sensors

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import numpy as np
from cv_bridge import CvBridge
import cv2
from copy import deepcopy

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Declare parameters
        self.declare_parameter('action_id', 0)
        self.declare_parameter('user_id', -1)  # -1 = unspecified
        
        # Get parameters
        self.action_id = self.get_parameter('action_id').get_parameter_value().integer_value
        self.user_id = self.get_parameter('user_id').get_parameter_value().integer_value
        
        # Data buffers
        self.current_detections = None
        self.current_yolo_image = None
        self.current_hand_finger = None
        self.current_hand_thumbs_up = None
        self.current_hand_image = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.yolo_detections_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.yolo_detections_callback,
            1  # Queue size 1
        )
        
        self.yolo_image_sub = self.create_subscription(
            Image,
            '/yolo/processed_image',
            self.yolo_image_callback,
            1  # Queue size 1
        )
        
        self.hand_finger_sub = self.create_subscription(
            PointStamped,
            '/hand/pointer_finger',
            self.hand_finger_callback,
            1  # Queue size 1
        )
        
        self.hand_image_sub = self.create_subscription(
            Image,
            '/hand/annotated_image',
            self.hand_image_callback,
            1  # Queue size 1
        )
        
        self.hand_thumbs_up_sub = self.create_subscription(
            PointStamped,
            '/hand/thumbs_up',
            self.hand_thumbs_up_callback,
            1  # Queue size 1
        )
        
        # Publisher
        self.fusion_pub = self.create_publisher(
            Detection2DArray,
            '/fusion_out',
            1  # Queue size 1
        )
        
        # Timer to clear old gestures (1 Hz)
        self.gesture_timeout = 0.3  # Clear gestures after 0.3 seconds
        self.last_finger_time = None
        self.last_thumbs_up_time = None
        self.create_timer(0.5, self.gesture_timeout_callback)
        
        self.get_logger().info('🔀 Sensor Fusion Node gestartet ✅')
        self.get_logger().info(f'Action ID: {self.action_id}')
        self.get_logger().info(f'User ID: {self.user_id if self.user_id != -1 else "unspecified"}')
        self.get_logger().info('Subscribed to: /yolo/detections, /yolo/processed_image, /hand/pointer_finger, /hand/thumbs_up, /hand/annotated_image')
        self.get_logger().info('Publishing to: /fusion_out')
    
    def yolo_detections_callback(self, msg):
        """Callback für YOLO Detections"""
        self.current_detections = msg
        self.process_and_publish()
    
    def yolo_image_callback(self, msg):
        """Callback für YOLO verarbeitetes Bild"""
        self.current_yolo_image = msg
        self.process_and_publish()
    
    def hand_finger_callback(self, msg):
        """Callback für Zeigefinger-Position"""
        self.current_hand_finger = msg
        self.last_finger_time = self.get_clock().now()
        self.process_and_publish()
    
    def hand_image_callback(self, msg):
        """Callback für Hand-annotiertes Bild"""
        self.current_hand_image = msg
        self.process_and_publish()
    
    def hand_thumbs_up_callback(self, msg):
        """Callback für Thumbs Up Position"""
        self.current_hand_thumbs_up = msg
        self.last_thumbs_up_time = self.get_clock().now()
        self.process_and_publish()
    
    def gesture_timeout_callback(self):
        """Clear gestures if they're older than timeout"""
        now = self.get_clock().now()
        
        # Clear finger if timeout
        if self.last_finger_time:
            age = (now - self.last_finger_time).nanoseconds / 1e9
            if age > self.gesture_timeout:
                self.current_hand_finger = None
                self.last_finger_time = None
        
        # Clear thumbs_up if timeout
        if self.last_thumbs_up_time:
            age = (now - self.last_thumbs_up_time).nanoseconds / 1e9
            if age > self.gesture_timeout:
                self.current_hand_thumbs_up = None
                self.last_thumbs_up_time = None
    
    def is_point_in_bbox(self, point_x, point_y, detection):
        """Prüft ob ein Punkt (x,y) innerhalb einer Detection-Bounding-Box liegt"""
        # Bounding Box aus Detection extrahieren
        center_x = detection.bbox.center.position.x
        center_y = detection.bbox.center.position.y
        size_x = detection.bbox.size_x
        size_y = detection.bbox.size_y
        
        # Bounding Box Grenzen berechnen
        x1 = center_x - size_x / 2
        y1 = center_y - size_y / 2
        x2 = center_x + size_x / 2
        y2 = center_y + size_y / 2
        
        # Prüfen ob Punkt innerhalb der Box liegt
        return x1 <= point_x <= x2 and y1 <= point_y <= y2
    
    def process_and_publish(self):
        """Haupt-Logik: Fused Data verarbeiten und publizieren"""
        if self.current_detections is None:
            return  # Warten bis alle Daten verfügbar sind
        
        # Create fused detection message
        fused_detection = Detection2DArray()
        fused_detection.header = self.current_detections.header
        fused_detection.header.stamp = self.get_clock().now().to_msg()
        
        # Determine action_id and user_id based on hand gesture and detection
        result_action_id = "default"
        result_user_id = self.user_id if self.user_id != -1 else None
        
        # Check for thumbs up gesture (overrides everything)
        if self.current_hand_thumbs_up:
            result_action_id = "sit"
            self.get_logger().info('👍 THUMBS UP detected → action_id=SIT')
        
        # Check if hand finger is detected (only if not thumbs up)
        elif self.current_hand_finger:
            finger_x = self.current_hand_finger.point.x
            finger_y = self.current_hand_finger.point.y
            
            # Check if finger is inside any detection bounding box
            for detection in self.current_detections.detections:
                if self.is_point_in_bbox(finger_x, finger_y, detection):
                    # Finger is pointing at a detected person!
                    result_action_id = "follow"
                    # Use YOLO tracking ID as user_id
                    if detection.id != "":
                        try:
                            result_user_id = int(detection.id)
                        except ValueError:
                            result_user_id = None
                    
                    self.get_logger().info(
                        f'✅ ACTION=FOLLOW: Zeigefinger zeigt auf Person '
                        f'(Tracking ID={detection.id}, User={result_user_id})'
                    )
                    break
        
        # Log fusion summary
        detected_count = len(self.current_detections.detections)
        
        if self.current_hand_thumbs_up:
            self.get_logger().info(
                f'🔀 FUSION: {detected_count} detections, '
                f'THUMBS UP detected, Action={result_action_id}'
            )
        elif self.current_hand_finger:
            self.get_logger().info(
                f'🔀 FUSION: {detected_count} detections, '
                f'Hand at ({self.current_hand_finger.point.x:.0f}, {self.current_hand_finger.point.y:.0f}), '
                f'Action={result_action_id}, User={result_user_id}'
            )
        else:
            self.get_logger().info(
                f'🔀 FUSION: {detected_count} detections, '
                f'no hand gesture, Action={result_action_id}, User={result_user_id}'
            )
        
        # Copy detections from YOLO and add metadata
        for detection in self.current_detections.detections:
            # Copy detection to avoid modifying original
            new_detection = deepcopy(detection)
            
            # Override detection ID with action_id and user_id
            # Format: "action_id:user_id" or "action_id" if no user_id
            if result_user_id is not None:
                new_detection.id = f"{result_action_id}:{result_user_id}"
            else:
                new_detection.id = result_action_id
            
            # Add to fused output
            fused_detection.detections.append(new_detection)
        
        # Publish fused data
        self.fusion_pub.publish(fused_detection)
    
    def create_fusion_summary(self):
        """Erstellt eine Zusammenfassung der fused Daten"""
        summary = {
            'action_id': self.action_id,
            'user_id': self.user_id,
            'timestamp': self.get_clock().now(),
            'yolo_detections': len(self.current_detections.detections) if self.current_detections else 0,
            'hand_finger_detected': self.current_hand_finger is not None,
            'hand_position': {
                'x': self.current_hand_finger.point.x if self.current_hand_finger else None,
                'y': self.current_hand_finger.point.y if self.current_hand_finger else None,
            }
        }
        return summary

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Sensor Fusion Node beendet 🛑')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

