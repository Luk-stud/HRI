#!/bin/bash
# stop_all_nodes.sh
# Stoppt alle laufenden Polygon Nav Nodes

echo "🛑 Stopping all polygon_nav nodes..."

# Finde alle running Nodes
NODES=(
    "yolo_processor"
    "person_tracker"
    "hand_gesture_detector"
    "sensor_fusion"
    "state_machine"
    "polygon_explorer"
)

for node in "${NODES[@]}"; do
    echo "Killing $node..."
    pkill -f "$node" 2>/dev/null && echo "✓ Stopped $node" || echo "  $node not running"
done

# Clean up any ROS2 processes
pkill -f "ros2 run polygon_nav" 2>/dev/null

echo "✅ All nodes stopped"

