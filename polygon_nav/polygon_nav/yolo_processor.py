#!/usr/bin/env python3
# yolo_processor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
from datetime import datetime

class YoloProcessor(Node):
    def __init__(self):
        super().__init__('yolo_processor')
        
        # YOLO Modell laden
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO Modell geladen')
        
        # CV Bridge für ROS-OpenCV Konvertierung
        self.bridge = CvBridge()
        
        # 🔽 SUBSCRIBER: Bildstream vom Input-Topic
        self.image_sub = self.create_subscription(
            Image,
            '/image',  # Input Topic (kann angepasst werden)
            self.image_callback,
            1  # Queue size 1
        )
        self.get_logger().info('Subscribed to /image')
        
        # 🔼 PUBLISHER: Detections auf Output-Topic
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',  # Output Topic für Detections
            1  # Queue size 1
        )
        
        # 🔼 PUBLISHER: Visualisiertes Bild
        self.image_pub = self.create_publisher(
            Image,
            '/yolo/processed_image',  # Output Topic für Bild mit Bounding Boxes
            1  # Queue size 1
        )
        
        # Logging
        self.detections_log = []
        self.log_file = f"detections_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        self.get_logger().info('YOLO Processor Node gestartet ✅')
        self.get_logger().info('Using BYTETrack for object tracking')

    def image_callback(self, msg):
        """📨 Empfängt Bild, verarbeitet mit YOLO, publiziert Resultate"""
        try:
            # 1. ROS Image → OpenCV konvertieren
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 2. 🚀 YOLO Inference mit Tracking
            # Use track() for object tracking with persistent IDs
            # BYTETrack is the default tracker in Ultralytics
            results = self.model.track(
                cv_image, 
                conf=0.5, 
                persist=True,  # Keep tracking across frames
                verbose=False
            )
            
            # 3. 📋 Detections erstellen
            detection_array = Detection2DArray(header=msg.header)
            detection_array.header = msg.header
            
            # 4. 🎨 Visualisiertes Bild erstellen
            annotated_frame = results[0].plot()
            
            person_count = 0
            tracked_persons = {}
            
            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls.item())
                    conf = box.conf.item()
                    label = result.names[cls_id]
                    
                    # Get tracking ID (None if not tracked yet)
                    track_id = int(box.id.item()) if box.id is not None else None
                    
                    if label == 'person' and conf > 0.3:
                        person_count += 1
                        
                        # Bounding Box Koordinaten
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        
                        # Detection erstellen
                        detection = Detection2D()
                        detection.bbox.center.position.x = (x1 + x2) / 2
                        detection.bbox.center.position.y = (y1 + y2) / 2
                        detection.bbox.size_x = x2 - x1
                        detection.bbox.size_y = y2 - y1
                        
                        # Hypothesis
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = "person"
                        hypothesis.hypothesis.score = float(conf)
                        detection.results.append(hypothesis)
                        
                        # Add tracking ID to detection (store in ID field)
                        if track_id is not None:
                            detection.id = str(track_id)
                            tracked_persons[track_id] = (x1, y1, x2, y2, conf)
                        
                        detection_array.detections.append(detection)
                        
                        # Logging mit Tracking ID
                        self.log_detection(msg.header, conf, [x1, y1, x2, y2], track_id)
            
            # 5. 📤 Resultate publizieren
            self.detection_pub.publish(detection_array)
            
            # Visualisiertes Bild publizieren
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
            
            if person_count > 0:
                track_info = f" (IDs: {list(tracked_persons.keys())})" if tracked_persons else ""
                self.get_logger().info(f'👤 {person_count} Person(en) erkannt{track_info}')
                        
        except Exception as e:
            self.get_logger().error(f'Verarbeitungsfehler: {e}')

    def log_detection(self, header, confidence, bbox, track_id=None):
        """Speichert Erkennungen in Log mit Tracking ID"""
        detection_data = {
            'timestamp': datetime.now().isoformat(),
            'confidence': float(confidence),
            'bbox': bbox,
            'frame_id': header.frame_id,
            'stamp_sec': header.stamp.sec,
            'stamp_nanosec': header.stamp.nanosec
        }
        
        # Add tracking ID if available
        if track_id is not None:
            detection_data['track_id'] = int(track_id)
        
        self.detections_log.append(detection_data)
        
        # Periodisch speichern
        if len(self.detections_log) % 10 == 0:
            self.save_log()

    def save_log(self):
        """Speichert Log in JSON Datei"""
        try:
            with open(self.log_file, 'w') as f:
                json.dump(self.detections_log, f, indent=2)
            self.get_logger().info(f'Log gespeichert: {self.log_file}')
        except Exception as e:
            self.get_logger().error(f'Log-Speicherfehler: {e}')

    def __del__(self):
        """Speichert Log beim Beenden"""
        self.save_log()

def main(args=None):
    rclpy.init(args=args)
    node = YoloProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('YOLO Processor beendet 🛑')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
