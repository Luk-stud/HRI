# ROS2 Polygon Navigation System - Architecture

## 📊 System Overview

Dieses Dokument beschreibt die Architektur des ROS2 Polygon Navigation Systems mit allen Nodes, Topics und deren Verknüpfungen.

## 🖼️ Visual Diagram

Das visuelle Architektur-Diagramm ist in `ros2_architecture.png` zu finden.

## 📦 Nodes

### 1. yolo_processor
**Node Name:** `yolo_processor`  
**Python File:** `polygon_nav/polygon_nav/yolo_processor.py`

**Subscribers:**
- `/image` (sensor_msgs/Image) - Kamera-Bildinput

**Publishers:**
- `/yolo/detections` (vision_msgs/Detection2DArray) - Personendetektionen mit Tracking-IDs
- `/yolo/processed_image` (sensor_msgs/Image) - Visualisiertes Bild mit Bounding Boxes

**Funktion:**
- Verwendet YOLO v8 für Personendetektion
- BYTETrack für robustes Multi-Object-Tracking
- Publiziert erkannte Personen mit persistenten Tracking-IDs
- Loggt Detections in JSON-Dateien

---

### 2. hand_gesture_detector
**Node Name:** `hand_gesture_detector`  
**Python File:** `polygon_nav/polygon_nav/hand_gesture_detector.py`

**Subscribers:**
- `/image` (sensor_msgs/Image) - Kamera-Bildinput

**Publishers:**
- `/hand/pointer_finger` (geometry_msgs/PointStamped) - Zeigefinger-Position (x, y, z)
- `/hand/annotated_image` (sensor_msgs/Image) - Bild mit Hand-Skeleton

**Funktion:**
- MediaPipe Hand-Tracking für Hand-Erkennung
- Überprüft ob Zeigefinger gehoben ist
- Publiziert Koordinaten wenn Zeigefinger erkannt wird

---

### 3. person_position_tracker
**Node Name:** `person_position_tracker`  
**Python File:** `polygon_nav/polygon_nav/person_position_tracker.py`

**Subscribers:**
- `/yolo/detections` (vision_msgs/Detection2DArray) - Personendetektionen
- `/odom` (nav_msgs/Odometry) - Robot-Position und Orientierung
- `/scan` (sensor_msgs/LaserScan) - LiDAR-Daten

**Publishers:**
- `/person_position` (geometry_msgs/PointStamped) - Welt-Position der Person
- `/person_marker` (geometry_msgs/PoseStamped) - Marker für RViz

**Funktion:**
- Berechnet Welt-Koordinaten der erkannten Personen
- Verwendet Kamera-Detection, LiDAR-Distanz und Odometrie
- Transformiert relative Position in Welt-Koordinaten (odom frame)

---

### 4. polygon_navigation_client
**Node Name:** `polygon_navigation_client`  
**Python File:** `polygon_nav/polygon_nav/polygon_nav_client.py`

**Action Client:**
- `navigate_to_pose` (nav2_msgs/action/NavigateToPose) - NAV2 Navigation Action

**Funktion:**
- Implementiert Polygon-basierte Coverage-Pfade
- Sendet Navigations-Ziele an Nav2
- Erstellt boustrophedonischen Pfad für vollständige Abdeckung

---

## 🔄 Topic Flow

```
Camera/Lidar → /image, /odom, /scan
     ↓
┌─────────────────────────────────────┐
│  EXTERNAL DATA SOURCES              │
│  • Camera Stream                     │
│  • Odometry                          │
│  • LiDAR Scan                        │
└─────────────────────────────────────┘
     ↓
┌─────────────────────────────────────┐
│  DETECTION LAYER                    │
│  • yolo_processor                   │
│    → /yolo/detections               │
│    → /yolo/processed_image          │
│  • hand_gesture_detector            │
│    → /hand/pointer_finger           │
│    → /hand/annotated_image          │
└─────────────────────────────────────┘
     ↓
┌─────────────────────────────────────┐
│  TRACKING LAYER                     │
│  • person_position_tracker          │
│    ← /yolo/detections               │
│    ← /odom                           │
│    ← /scan                           │
│    → /person_position                │
│    → /person_marker                  │
└─────────────────────────────────────┘
     ↓
┌─────────────────────────────────────┐
│  NAVIGATION LAYER                   │
│  • polygon_navigation_client         │
│    → navigate_to_pose Action         │
└─────────────────────────────────────┘
```

## 📊 Message Types

### Input Topics (External)
| Topic | Type | Description |
|-------|------|-------------|
| `/image` | sensor_msgs/Image | Kamera-Bildstream |
| `/odom` | nav_msgs/Odometry | Robot-Odometrie und Orientierung |
| `/scan` | sensor_msgs/LaserScan | LiDAR-Scandaten |

### Output Topics (Internal)
| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/yolo/detections` | vision_msgs/Detection2DArray | yolo_processor | Personendetektionen mit Tracking |
| `/yolo/processed_image` | sensor_msgs/Image | yolo_processor | Visualisiertes YOLO-Bild |
| `/hand/pointer_finger` | geometry_msgs/PointStamped | hand_gesture_detector | Zeigefinger-Koordinaten |
| `/hand/annotated_image` | sensor_msgs/Image | hand_gesture_detector | Bild mit Hand-Skeleton |
| `/person_position` | geometry_msgs/PointStamped | person_position_tracker | Welt-Position der Person |
| `/person_marker` | geometry_msgs/PoseStamped | person_position_tracker | RViz Marker |

## 🚀 Usage

### Start Detection Nodes
```bash
# Terminal 1: YOLO Processor
ros2 run polygon_nav yolo_processor

# Terminal 2: Hand Gesture Detector
ros2 run polygon_nav hand_gesture_detector

# Terminal 3: Person Position Tracker
ros2 run polygon_nav person_tracker
```

### Visualize in RViz
```bash
ros2 run rviz2 rviz2
```

**Display Topics:**
- `/yolo/processed_image` - YOLO Visualisierung
- `/hand/annotated_image` - Hand-Tracking Visualisierung
- `/person_marker` - Person Position Marker

### Monitor Topics
```bash
# Zeigefinger-Position
ros2 topic echo /hand/pointer_finger

# Person Positionen
ros2 topic echo /person_position

# YOLO Detections
ros2 topic echo /yolo/detections
```

## 🔧 Dependencies

### System Packages
- `ros-humble-desktop`
- `python3-opencv`
- `cv_bridge`
- `sensor_msgs`
- `geometry_msgs`
- `vision_msgs`
- `nav2_msgs`

### Python Packages
- `ultralytics` (YOLO)
- `mediapipe` (Hand Tracking)
- `numpy`
- `cv_bridge`
- `shapely` (Polygon Navigation)

## 📝 Notes

- **Coordinate Frames:** Alle Positionen werden im `odom` frame berechnet
- **Tracking IDs:** YOLO BYTETrack vergibt persistente IDs für erkannte Personen
- **Real-time Processing:** Alle Nodes arbeiten asynchron und unabhängig
- **Modularity:** Jeder Node kann einzeln gestartet/gestoppt werden

## 🎯 Workflow

1. **Image Input** → Camera publiziert auf `/image`
2. **YOLO Detection** → Erkennt Personen, vergibt Tracking-IDs
3. **Hand Detection** → Erkennt Zeigefinger-Gesten
4. **Position Tracking** → Berechnet Welt-Koordinaten der Personen
5. **Navigation** → Navigiert zu Zielpositionen

