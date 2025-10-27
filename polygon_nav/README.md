# Polygon Navigation System

Automatisches ROS2-System für Person-Tracking, Hand-Gestures und Navigation.

## 🚀 Quick Start

### Starte alle Nodes automatisch:

```bash
cd /home/user/ROS2/polygon_nav
./start_all_nodes.sh
```

Das Script:
1. ✅ Baut das Package mit `colcon build`
2. ✅ Sourced das Workspace
3. ✅ Startet automatisch alle Nodes aus `setup.py`
4. ✅ Zeigt Status aller laufenden Nodes

### Node Status checken:

```bash
./check_nodes.sh
```

### Alle Nodes stoppen:

```bash
./stop_all_nodes.sh
```

## 📋 Verfügbare Nodes

| Node | Start Command | Description |
|------|-------------|-------------|
| `yolo_processor` | `ros2 run polygon_nav yolo_processor` | YOLO Person Detection mit BYTETrack |
| `hand_gesture_detector` | `ros2 run polygon_nav hand_gesture_detector` | MediaPipe Hand-Tracking |
| `person_tracker` | `ros2 run polygon_nav person_tracker` | Person Position Tracking |
| `sensor_fusion` | `ros2 run polygon_nav sensor_fusion` | Sensor Fusion (Follow/Sit Detection) |
| `state_machine` | `ros2 run polygon_nav state_machine` | State Machine (SIT/FOLLOW) |
| `polygon_explorer` | `ros2 run polygon_nav polygon_explorer` | Navigation Client |

## 🔄 Workflow

```
Camera Stream
     ↓
┌────────────────────────────────┐
│ DETECTION LAYER                │
│ • yolo_processor               │
│ • hand_gesture_detector        │
└────────────────────────────────┘
     ↓
┌────────────────────────────────┐
│ SENSOR FUSION                  │
│ • sensor_fusion                │
│   → action_id: "follow"/"sit" │
└────────────────────────────────┘
     ↓
┌────────────────────────────────┐
│ STATE MACHINE                  │
│ • state_machine                │
│   → State: SIT or FOLLOW       │
└────────────────────────────────┘
```

## 📊 Topics

### Input Topics (External)
- `/image` (Camera Stream)
- `/odom` (Robot Odometry)
- `/scan` (LiDAR Scan)

### Internal Topics
- `/yolo/detections` - Person Detections
- `/yolo/processed_image` - YOLO Visual
- `/hand/pointer_finger` - Hand Gesture Position
- `/hand/annotated_image` - Hand Visual
- `/fusion_out` - Fused Data
- `/state_machine_out` - State Machine Output
- `/person_position` - World Position of Persons

## 🔧 Manual Start (Advanced)

Wenn Sie Nodes einzeln starten möchten:

```bash
# Terminal 1: YOLO
ros2 run polygon_nav yolo_processor

# Terminal 2: Hand Gesture
ros2 run polygon_nav hand_gesture_detector

# Terminal 3: Person Tracker
ros2 run polygon_nav person_tracker

# Terminal 4: Sensor Fusion
ros2 run polygon_nav sensor_fusion --ros-args -p action_id:=0

# Terminal 5: State Machine
ros2 run polygon_nav state_machine --ros-args -p debug:=true
```

## 📝 Logs

Node-Logs werden in `/tmp/<node_name>.log` gespeichert:

```bash
# Logs ansehen
tail -f /tmp/yolo_processor.log
tail -f /tmp/hand_gesture_detector.log
```

## 🎯 Use Cases

### Follow a Person
1. System erkennt Personen (YOLO)
2. User zeigt auf Person (Hand Gesture)
3. Sensor Fusion → `action_id="follow"`
4. State Machine → State = FOLLOW
5. Robot folgt der Person

### Sit/Stop
1. User zeigt weg → `action_id="default"`
2. State Machine → State = SIT
3. Robot bleibt stehen

## 📸 Visualization

RViz2 Start:
```bash
ros2 run rviz2 rviz2
```

Display Topics:
- `/yolo/processed_image` - YOLO Detections
- `/hand/annotated_image` - Hand Tracking
- `/person_position` - Person Position Marker

