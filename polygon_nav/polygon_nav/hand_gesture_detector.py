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
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.image_sub = self.create_subscription(
            Image,
            '/image',  # Input topic
            self.image_callback,
            qos_profile
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
        
        # Declare parameters for performance / output control
        self.declare_parameter('processing_width', 640)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('annotation_decimation', 1)
        self.processing_width = int(self.get_parameter('processing_width').value)
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self.annotation_decimation = max(1, int(self.get_parameter('annotation_decimation').value))
        self.annotation_counter = 0

        # Publisher für annotiertes Bild (optional)
        if self.publish_annotated:
            self.image_pub = self.create_publisher(
                Image,
                '/hand/annotated_image',
                1  # Queue size 1
            )
        # Declare parameters to tune robustness without editing code
        self.declare_parameter('min_detection_frames', 1)
        self.declare_parameter('finger_extension_ratio', 0.18)
        self.declare_parameter('thumb_vertical_ratio', 0.12)
        self.declare_parameter('fold_margin_ratio', 0.04)

        self.min_detection_frames = max(1, int(self.get_parameter('min_detection_frames').value))
        self.finger_extension_ratio = float(self.get_parameter('finger_extension_ratio').value)
        self.thumb_vertical_ratio = float(self.get_parameter('thumb_vertical_ratio').value)
        self.fold_margin_ratio = float(self.get_parameter('fold_margin_ratio').value)

        # Simple temporal smoothing to reduce flicker between finger/thumb detections
        self.finger_detect_counter = 0
        self.thumb_detect_counter = 0
        self.finger_candidate = None
        self.thumb_candidate = None
        
        self.get_logger().info('👋 Hand Gesture Detector gestartet ✅')
        self.get_logger().info('Subscribed to /image')
        self.get_logger().info('Publishing to /hand/pointer_finger and /hand/thumbs_up')
    
    def estimate_hand_scale(self, landmarks):
        """Estimate hand scale based on wrist to middle finger MCP distance."""
        wrist = landmarks[self.mp_hands.HandLandmark.WRIST]
        middle_mcp = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        dx = wrist.x - middle_mcp.x
        dy = wrist.y - middle_mcp.y
        dz = wrist.z - middle_mcp.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) + 1e-6

    def is_index_finger_up(self, landmarks, hand_scale):
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
        index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]

        extension_threshold = self.finger_extension_ratio * hand_scale
        # Überprüfe, dass Finger deutlich gestreckt ist (Tip deutlich über PIP & MCP)
        tip_above_pip = (index_pip.y - index_tip.y) > extension_threshold
        tip_above_mcp = (index_mcp.y - index_tip.y) > extension_threshold
        # Finger soll nahezu gerade sein -> Abstand zwischen Tip und PIP größer als zwischen PIP und MCP
        pip_to_tip = abs(index_pip.y - index_tip.y)
        mcp_to_pip = abs(index_mcp.y - index_pip.y)
        straight_enough = pip_to_tip > (0.7 * mcp_to_pip)
        
        is_up = tip_above_pip and tip_above_mcp and straight_enough
        return is_up, index_tip
    
    def is_thumbs_up(self, landmarks, hand_scale):
        """
        Überprüft ob der Daumen gehoben ist
        Thumbs Up Erkennung: Daumen ausgebreitet und nach oben
        """
        # Daumen-Punkte
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        thumb_mcp = landmarks[self.mp_hands.HandLandmark.THUMB_MCP]
        thumb_cmc = landmarks[self.mp_hands.HandLandmark.THUMB_CMC]
        
        vertical_threshold = self.thumb_vertical_ratio * hand_scale

        # Prüfe ob Daumen nach oben zeigt (y abnehmend) und weitgehend vertikal verläuft
        thumb_vec_y = thumb_ip.y - thumb_tip.y  # positiv wenn Tip deutlich über IP
        thumb_vec_x = abs(thumb_tip.x - thumb_ip.x)
        vertical_enough = thumb_vec_y > vertical_threshold and thumb_vec_y > thumb_vec_x
        thumb_up = vertical_enough and (thumb_tip.y < thumb_ip.y < thumb_mcp.y)
        
        # Alle anderen Finger sollten unten sein (closed fist oder gerade)
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        pinky_tip = landmarks[self.mp_hands.HandLandmark.PINKY_TIP]
        
        # Finger sollten niedriger als MCP sein (gefaltet)
        fold_margin = self.fold_margin_ratio * hand_scale
        fingers_down = (
            index_tip.y > landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y - fold_margin and
            middle_tip.y > landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y - fold_margin and
            ring_tip.y > landmarks[self.mp_hands.HandLandmark.RING_FINGER_MCP].y - fold_margin and
            pinky_tip.y > landmarks[self.mp_hands.HandLandmark.PINKY_MCP].y - fold_margin
        )
        
        is_thumbs_up = thumb_up and fingers_down
        
        return is_thumbs_up, thumb_tip
    
    def image_callback(self, msg):
        """Verarbeitet eingehende Bilder"""
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Optional downscale for faster processing
            orig_h, orig_w = cv_image.shape[:2]
            proc_image = cv_image
            if self.processing_width > 0 and orig_w > self.processing_width:
                scale = self.processing_width / float(orig_w)
                target_size = (self.processing_width, max(1, int(orig_h * scale)))
                proc_image = cv2.resize(cv_image, target_size, interpolation=cv2.INTER_LINEAR)
            else:
                scale = 1.0

            # Convert BGR to RGB für MediaPipe
            rgb_image = cv2.cvtColor(proc_image, cv2.COLOR_BGR2RGB)
            
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
                    hand_scale = self.estimate_hand_scale(landmarks)
                    is_up, index_tip = self.is_index_finger_up(landmarks, hand_scale)
                    thumbs_up, thumb_tip = self.is_thumbs_up(landmarks, hand_scale)
                    
                    # Get image dimensions
                    h, w = cv_image.shape[:2]
                    
                    if is_up:
                        # Convert normalized coordinates to pixel coordinates
                        self.finger_candidate = (
                            int(index_tip.x * w),
                            int(index_tip.y * h),
                            index_tip.z
                        )
                        self.finger_detect_counter += 1
                    else:
                        self.finger_detect_counter = 0
                        self.finger_candidate = None

                    if self.finger_detect_counter >= self.min_detection_frames and self.finger_candidate:
                        finger_detected = True
                        finger_position = self.finger_candidate
                        
                        # Zeichne Zeigefinger-Tip
                        cv2.circle(annotated_image, (finger_position[0], finger_position[1]), 10, (0, 255, 255), -1)
                        
                        self.get_logger().info(
                            f'👆 Zeigefinger erkannt: x={finger_position[0]}, y={finger_position[1]}'
                        )
                    
                    if thumbs_up:
                        # Convert normalized coordinates to pixel coordinates
                        self.thumb_candidate = (
                            int(thumb_tip.x * w),
                            int(thumb_tip.y * h),
                            thumb_tip.z
                        )
                        self.thumb_detect_counter += 1
                    else:
                        self.thumb_detect_counter = 0
                        self.thumb_candidate = None

                    if self.thumb_detect_counter >= self.min_detection_frames and self.thumb_candidate:
                        thumbs_up_detected = True
                        thumbs_up_position = self.thumb_candidate
                        
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
            
            if self.publish_annotated:
                self.annotation_counter = (self.annotation_counter + 1) % self.annotation_decimation
                if self.annotation_counter == 0:
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

