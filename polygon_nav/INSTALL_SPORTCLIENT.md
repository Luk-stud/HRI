# Unitree Legged SDK Installation

## Installation als ROS2 Package

### Schritt 1: SDK Repository klonen (falls noch nicht vorhanden)

```bash
cd ~/ROS2
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
```

### Schritt 2: Python Wrapper bauen

Das SDK hat Python-Bindings, die mit CMake gebaut werden müssen:

```bash
cd ~/ROS2/unitree_legged_sdk
mkdir -p build
cd build

# Python Wrapper aktivieren und bauen
cmake .. -DPYTHON_BUILD=ON
make -j$(nproc)
```

Die Python-Bindings werden in `lib/python/amd64/` (oder `arm64/` auf ARM) erstellt.

### Schritt 3: Mit colcon bauen (optional, für ROS2 Integration)

```bash
cd ~/ROS2
colcon build --packages-select unitree_legged_sdk
source install/setup.bash
```

### Schritt 4: Python-Pfad konfigurieren

Das SDK muss im Python-Pfad verfügbar sein. Füge in `sit_node.py` oder systemweit hinzu:

```python
import sys
sys.path.append('/home/user/ROS2/unitree_legged_sdk/lib/python/amd64')
import robot_interface as sdk
```

Oder setze die Umgebungsvariable:

```bash
export PYTHONPATH=$PYTHONPATH:/home/user/ROS2/unitree_legged_sdk/lib/python/amd64
```

### Schritt 5: Workspace neu bauen

```bash
cd ~/ROS2
colcon build --packages-select polygon_nav
source install/setup.bash
```

## Prüfen ob Installation erfolgreich

```bash
python3 -c "import sys; sys.path.append('/home/user/ROS2/unitree_legged_sdk/lib/python/amd64'); import robot_interface as sdk; print('✅ SDK verfügbar!')"
```

## SDK API Verwendung

Das SDK verwendet `robot_interface`, nicht `SportClient`:

```python
import robot_interface as sdk

HIGHLEVEL = 0xee
udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)
cmd = sdk.HighCmd()
state = sdk.HighState()

# Sit: cmd.mode = 5
# Stand: cmd.mode = 6
# Idle: cmd.mode = 0
```

## Hinweis

Der `sit_node` verwendet standardmäßig den ROS2 Service `/set_mode`, der bereits funktioniert. 
Falls das SDK verfügbar ist, wird es automatisch verwendet (siehe `sit_node.py`).

