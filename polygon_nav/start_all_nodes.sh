#!/bin/bash
# start_all_nodes.sh
# Automatisches Build und Start aller Polygon Nav Nodes

set -e

WORKSPACE_ROOT="/home/user/ROS2"

echo "🚀 Starting Polygon Nav System"
echo "================================"

# 1. Build
cd "$WORKSPACE_ROOT"
echo "📦 Building package..."
colcon build --packages-select polygon_nav

echo "✅ Build successful"

# 2. Source
echo "🔧 Sourcing workspace..."
source "$WORKSPACE_ROOT/install/setup.bash"

# 3. Extract node names from setup.py
SCRIPT_DIR="/home/user/ROS2/polygon_nav"
cd "$SCRIPT_DIR"

NODES=$(python3 << 'PYTHON_EOF'
import re
with open('setup.py', 'r') as f:
    content = f.read()
    pattern = r"'(\w+)\s+=\s+\w+\.\w+:main'"
    matches = re.findall(pattern, content)
    for match in matches:
        print(match)
PYTHON_EOF
)

echo "Found nodes: $NODES"
echo ""



# 4. Start all nodes (except vosk_voice_assistant and state_machine which are started separately)
for node in $NODES; do
    # Skip vosk_voice_assistant and state_machine as they're started separately at the end
    if [ "$node" != "vosk_voice_assistant" ] && [ "$node" != "state_machine" ]; then
        echo "▶ Starting: ros2 run polygon_nav $node"
        ros2 run polygon_nav $node &
    fi
done

sleep 5
echo "▶ Starting: ros2 run polygon_nav state_machine"
ros2 run polygon_nav state_machine &

# start the vosk voice assistant at the end
echo "▶ Starting: ros2 run polygon_nav vosk_voice_assistant"
ros2 run polygon_nav vosk_voice_assistant &

echo ""
echo "✅ All nodes started"
echo "Press Ctrl+C to stop"

wait

