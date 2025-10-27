#!/usr/bin/env python3
# hand_gesture_detector.py
# MediaPipe Hand-Tracking Node für Zeigefinger-Erkennung

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np

class HandGestureDetector(Node):
    def __init__(self):
        super().__init__('hand_gesture_detector')
        
        # MediaPipe Hand-Lösung
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber für Bildstream
        self.image_sub = self.create_subscription(
            Image,
            '/image',  # Input topic
            self.image_callback,
            1  # Queue size 1
        )
        
        # Publisher für Zeigefinger-Position
        self.finger_pub = self.create_publisher(
            PointStamped,
            '/hand/pointer_finger',  # Output topic
            1  # Queue size 1
        )
        
        # Publisher für Thumbs Up Position
        self.thumbs_up_pub = self.create_publisher(
            PointStamped,
            '/hand/thumbs_up',  # Output topic for thumbs up gesture
            1  # Queue size 1
        )
        
        # Publisher für annotiertes Bild (optional)
        self.image_pub = self.create_publisher(
            Image,
            '/hand/annotated_image',
            1  # Queue size 1
        )
        
        self.get_logger().info('👋 Hand Gesture Detector gestartet ✅')
        self.get_logger().info('Subscribed to /image')
        self.get_logger().info('Publishing to /hand/pointer_finger and /hand/thumbs_up')
    
    def is_index_finger_up(self, landmarks):
        """
        Überprüft ob der Zeigefinger gehoben ist
        MediaPipe Landmark IDs:
        - WRIST: 0
        - THUMB_CMC, THUMB_MCP, THUMB_IP, THUMB_TIP: 1-4
        - INDEX_FINGER_MCP, INDEX_FINGER_PIP, INDEX_FINGER_DIP, INDEX_FINGER_TIP: 5-8
        """
        # Zeigefinger-Punkte
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        index_pip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]
        
        # Überprüfe ob Tip über PIP ist
        is_up = index_tip.y < index_pip.y
        
        return is_up, index_tip
    
    def is_thumbs_up(self, landmarks):
        """
        Überprüft ob der Daumen gehoben ist
        Thumbs Up Erkennung: Daumen ausgebreitet und nach oben
        """
        # Daumen-Punkte
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        thumb_mcp = landmarks[self.mp_hands.HandLandmark.THUMB_MCP]
        thumb_cmc = landmarks[self.mp_hands.HandLandmark.THUMB_CMC]
        
        # Prüfe ob Daumen nach oben zeigt (y abnehmend)
        thumb_up = (thumb_tip.y < thumb_ip.y < thumb_mcp.y)
        
        # Alle anderen Finger sollten unten sein (closed fist oder gerade)
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        
        # Finger sollten niedriger als MCP sein (gefaltet)
        fingers_down = (index_tip.y > landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y and
                       middle_tip.y > landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y)
        
        is_thumbs_up = thumb_up and fingers_down
        
        return is_thumbs_up, thumb_tip
    
    def image_callback(self, msg):
        """Verarbeitet eingehende Bilder"""
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Convert BGR to RGB für MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe
            results = self.hands.process(rgb_image)
            
            # Zeichne Resultate auf Bild
            annotated_image = cv_image.copy()
            
            finger_detected = False
            finger_position = None
            thumbs_up_detected = False
            thumbs_up_position = None
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Zeichne Hand-Skeleton
                    self.mp_draw.draw_landmarks(
                        annotated_image,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_draw.DrawingSpec(color=(0, 255, 0), thickness=2),
                        self.mp_draw.DrawingSpec(color=(255, 0, 0), thickness=2)
                    )
                    
                    # Überprüfe Zeigefinger und Thumbs Up
                    landmarks = hand_landmarks.landmark
                    is_up, index_tip = self.is_index_finger_up(landmarks)
                    thumbs_up, thumb_tip = self.is_thumbs_up(landmarks)
                    
                    # Get image dimensions
                    h, w = cv_image.shape[:2]
                    
                    if is_up:
                        finger_detected = True
                        
                        # Convert normalized coordinates to pixel coordinates
                        finger_position = (
                            int(index_tip.x * w),
                            int(index_tip.y * h),
                            index_tip.z
                        )
                        
                        # Zeichne Zeigefinger-Tip
                        cv2.circle(annotated_image, (finger_position[0], finger_position[1]), 10, (0, 255, 255), -1)
                        
                        self.get_logger().info(
                            f'👆 Zeigefinger erkannt: x={finger_position[0]}, y={finger_position[1]}'
                        )
                    
                    if thumbs_up:
                        thumbs_up_detected = True
                        
                        # Convert normalized coordinates to pixel coordinates
                        thumbs_up_position = (
                            int(thumb_tip.x * w),
                            int(thumb_tip.y * h),
                            thumb_tip.z
                        )
                        
                        # Zeichne Thumbs Up
                        cv2.circle(annotated_image, (thumbs_up_position[0], thumbs_up_position[1]), 15, (255, 255, 0), -1)
                        
                        self.get_logger().info(
                            f'👍 Thumbs Up erkannt: x={thumbs_up_position[0]}, y={thumbs_up_position[1]}'
                        )
            
            # Publish finger position if detected
            if finger_detected and finger_position:
                point_msg = PointStamped()
                point_msg.header.stamp = msg.header.stamp
                point_msg.header.frame_id = msg.header.frame_id
                point_msg.point.x = float(finger_position[0])
                point_msg.point.y = float(finger_position[1])
                point_msg.point.z = float(finger_position[2])  # Tiefe
                
                self.finger_pub.publish(point_msg)
            
            # Publish thumbs up position if detected
            if thumbs_up_detected and thumbs_up_position:
                point_msg = PointStamped()
                point_msg.header.stamp = msg.header.stamp
                point_msg.header.frame_id = msg.header.frame_id
                point_msg.point.x = float(thumbs_up_position[0])
                point_msg.point.y = float(thumbs_up_position[1])
                point_msg.point.z = float(thumbs_up_position[2])  # Tiefe
                
                self.thumbs_up_pub.publish(point_msg)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Fehler bei Bildverarbeitung: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = HandGestureDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Hand Gesture Detector beendet 🛑')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

