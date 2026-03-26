# Telepítési és Indítási Útmutató

## Követelmények

- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- Gazebo Fortress
- Python 3.10+

---

## 1. ROS2 Humble telepítése (ha még nincs)

```bash
# ROS2 GPG kulcs és forrás hozzáadása
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# Telepítés
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-gz -y

# Automatikus source (tedd a ~/.bashrc-be)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Gazebo Fortress telepítése

```bash
sudo apt install gz-fortress -y
```

## 3. ROS2 extra csomagok

```bash
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-ur-description \
  ros-humble-robot-state-publisher \
  python3-colcon-common-extensions
```

## 4. Python függőségek

```bash
pip install opencv-python numpy pytest
```

## 5. Workspace buildelése

```bash
# A visual_servoing_ws mappában:
cd visual_servoing_ws

# Függőségek automatikus telepítése
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Workspace aktiválása (minden új terminálban szükséges!)
source install/setup.bash
```

---

## Indítás

### Szimuláció (Gazebo + összes node)

```bash
# Terminál 1 – minden egyszerre:
ros2 launch visual_servoing simulation.launch.py
```

Ez elindítja:
- Gazebo (3D világ: fal, aljzat, kamera)
- robot_state_publisher
- ROS2-Gazebo híd
- vision_node
- controller_node
- RViz2

### Csak tesztelés (ROS2 nélkül)

```bash
cd visual_servoing_ws/src/visual_servoing
pytest test/test_ibvs.py -v
```

---

## Kalibrálás (első futtatás előtt)

A rendszer kalibrálás nélkül egyszerű középpont-alapú hibával működik.
A homográfia kalibráláshoz:

```bash
# 1. Robot mozgatása az ismert referencia pozícióba (manuálisan)

# 2. Kalibrációs parancs kiadása
ros2 service call /vision/calibrate std_srvs/srv/Trigger
```

---

## Hasznos parancsok

```bash
# Topic-ok listája
ros2 topic list

# Hibavektor figyelése valós időben
ros2 topic echo /vision/visual_error

# Állapotgép figyelése
ros2 topic echo /assembly/status

# Debug kép megjelenítése
ros2 run rqt_image_view rqt_image_view /vision/debug_image

# RViz2 önálló indítása
rviz2
```

---

## Mappastruktúra

```
visual_servoing_ws/
└── src/
    └── visual_servoing/
        ├── visual_servoing/
        │   ├── feature_detector.py   ← OpenCV detektálás
        │   ├── ibvs_controller.py    ← IBVS matematika
        │   ├── adrc_controller.py    ← ADRC vezérlő
        │   ├── vision_node.py        ← ROS2 vision node
        │   ├── controller_node.py    ← ROS2 vezérlő node
        │   └── camera_node.py        ← Valódi kamera (laborhoz)
        ├── config/
        │   └── params.yaml           ← Hangolható paraméterek
        ├── launch/
        │   └── simulation.launch.py  ← Teljes rendszer indítása
        ├── worlds/
        │   └── assembly_world.world  ← Gazebo világ
        ├── test/
        │   └── test_ibvs.py          ← Egységtesztek
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```
