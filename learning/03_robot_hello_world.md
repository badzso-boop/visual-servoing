# 03 – Robot Hello World: Kétcsuklós kar mozgatása

**Előfeltétel:** [02_gazebo_elso_lepesek.md](02_gazebo_elso_lepesek.md) kész

**Cél:** Egy egyszerű 2-csuklós robotkart betölteni Gazeboba, majd ROS2-ből mozgatni egyik pozícióból a másikba.

---

## 1. Mi az URDF?

Az URDF (Unified Robot Description Format) az XML alapú robot leíró fájl.

```
URDF ~ UR robotka konfigurációs fájl, de teljes 3D modell + kinematika
```

Egy robot felépítése URDF-ben:
```
robot
├── link: base_link           ← az alap (rögzített)
├── joint: joint1             ← forgó csukló 1
├── link: link1               ← az első kar szegmens
├── joint: joint2             ← forgó csukló 2
└── link: link2               ← a második kar szegmens (effector)
```

---

## 2. Egyszerű 2-csuklós kar létrehozása

```bash
mkdir -p ~/gazebo_tanulas/robot_ws/src/karcska_robot/urdf
nano ~/gazebo_tanulas/robot_ws/src/karcska_robot/urdf/karcska.urdf
```

```xml
<?xml version="1.0"?>
<robot name="karcska">

  <!-- BASE LINK: az alap, ami a talajhoz rögzített -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="szurke">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- 1. CSUKLÓ: base_link → link1 (forgás Z tengely körül) -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05"/>        <!-- a base_link tetején -->
    <axis xyz="0 0 1"/>             <!-- Z tengely körül forog -->
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>
  </joint>

  <!-- LINK1: az első kar szegmens (0.3m hosszú) -->
  <link name="link1">
    <visual>
      <origin xyz="0 0 0.15"/>      <!-- középen van a vizuális -->
      <geometry>
        <box size="0.04 0.04 0.3"/>
      </geometry>
      <material name="kek">
        <color rgba="0.2 0.4 0.8 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <box size="0.04 0.04 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.15"/>
      <mass value="0.5"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- 2. CSUKLÓ: link1 → link2 (forgás Y tengely körül = "könyök") -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.3"/>         <!-- link1 végén -->
    <axis xyz="0 1 0"/>             <!-- Y tengely körül (könyök) -->
    <limit lower="-2.0" upper="2.0" effort="10" velocity="1.0"/>
  </joint>

  <!-- LINK2: a második kar szegmens (0.25m hosszú) -->
  <link name="link2">
    <visual>
      <origin xyz="0 0 0.125"/>
      <geometry>
        <box size="0.035 0.035 0.25"/>
      </geometry>
      <material name="zold">
        <color rgba="0.2 0.7 0.3 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.125"/>
      <geometry>
        <box size="0.035 0.035 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.125"/>
      <mass value="0.3"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- TCP LINK: a tool center point (effector vége) -->
  <joint name="tcp_joint" type="fixed">
    <parent link="link2"/>
    <child link="tcp_link"/>
    <origin xyz="0 0 0.25"/>
  </joint>

  <link name="tcp_link">
    <visual>
      <geometry><sphere radius="0.02"/></geometry>
      <material name="piros"><color rgba="1 0 0 1"/></material>
    </visual>
  </link>

</robot>
```

---

## 3. ROS2 Package a robothoz

```bash
cd ~/gazebo_tanulas/robot_ws/src
ros2 pkg create --build-type ament_python karcska_robot \
  --dependencies rclpy sensor_msgs std_msgs
```

Hozd létre a szükséges mappákat:
```bash
mkdir -p ~/gazebo_tanulas/robot_ws/src/karcska_robot/urdf
mkdir -p ~/gazebo_tanulas/robot_ws/src/karcska_robot/launch
# Az URDF fájlt már létrehoztuk, mozgasd oda:
mv ~/gazebo_tanulas/robot_ws/src/karcska_robot/urdf/karcska.urdf \
   ~/gazebo_tanulas/robot_ws/src/karcska_robot/urdf/ 2>/dev/null || true
```

---

## 4. Launch fájl – robot betöltése Gazeboba

```bash
nano ~/gazebo_tanulas/robot_ws/src/karcska_robot/launch/robot_launch.py
```

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('karcska_robot')
    urdf_file = os.path.join(pkg, 'urdf', 'karcska.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # 1. Gazebo elindítása
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # 2. Robot state publisher (URDF → /robot_description topic)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # 3. Spawn: betölti a robotot Gazeboba
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'karcska',
                '-topic', '/robot_description',
                '-z', '0.05'        # kis magasságban indul
            ],
            output='screen'
        ),

    ])
```

---

## 5. setup.py módosítása

```bash
nano ~/gazebo_tanulas/robot_ws/src/karcska_robot/setup.py
```

```python
from setuptools import setup
import os
from glob import glob

package_name = 'karcska_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='te_neved',
    maintainer_email='email@example.com',
    description='Karcska robot tanuláshoz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mozgato = karcska_robot.mozgato_node:main',
        ],
    },
)
```

---

## 6. Mozgató node – A tényleges "Hello World"

```bash
nano ~/gazebo_tanulas/robot_ws/src/karcska_robot/karcska_robot/mozgato_node.py
```

```python
"""
Mozgato Node – Robot Hello World

Ez a node:
1. Küldi a joint pozíciókat a /joint_states topicra
2. Mozgatja a robotot A pozícióból B pozícióba
3. Vár, majd visszamegy

Analógia UR-ből:
  MoveJ [joint1=0, joint2=0]  →  rclpy.spin + timer_callback
  Waypoint A                  →  pos_a = [0.0, 0.0]
  Waypoint B                  →  pos_b = [1.57, -1.0]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class MozgatoNode(Node):
    def __init__(self):
        super().__init__('mozgato_node')

        # Publisher: joint parancsokat küldi
        # (a /joint_group_position_controller/commands topicra)
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Pozíciók radiánban [joint1, joint2]
        self.poz_a = [0.0, 0.0]               # egyenesen felfelé
        self.poz_b = [math.pi/2, -math.pi/4]  # 90° és -45°

        self.jelenlegi_poz = 'A'
        self.varakozas = 0

        # Minden 2 másodpercben vált
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Mozgato node elindult!')
        self.kuldd(self.poz_a)

    def kuldd(self, pozicio):
        msg = Float64MultiArray()
        msg.data = pozicio
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Mozgas ide: joint1={math.degrees(pozicio[0]):.1f}°, '
            f'joint2={math.degrees(pozicio[1]):.1f}°'
        )

    def timer_callback(self):
        if self.jelenlegi_poz == 'A':
            self.get_logger().info('--- Mozgas: A → B ---')
            self.kuldd(self.poz_b)
            self.jelenlegi_poz = 'B'
        else:
            self.get_logger().info('--- Mozgas: B → A ---')
            self.kuldd(self.poz_a)
            self.jelenlegi_poz = 'A'

def main():
    rclpy.init()
    node = MozgatoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 7. Build és futtatás

```bash
cd ~/gazebo_tanulas/robot_ws

# Függőségek telepítése
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Betöltés
source install/setup.bash

# Terminál 1: Robot betöltése
ros2 launch karcska_robot robot_launch.py

# Terminál 2 (várj amíg Gazebo betölt): Vezérlő
ros2 run karcska_robot mozgato
```

---

## 8. Amit látsz

A terminálban:
```
[mozgato_node]: Mozgato node elindult!
[mozgato_node]: Mozgas ide: joint1=0.0°, joint2=0.0°
[mozgato_node]: --- Mozgas: A → B ---
[mozgato_node]: Mozgas ide: joint1=90.0°, joint2=-45.0°
[mozgato_node]: --- Mozgas: B → A ---
...
```

A Gazebo ablakban: a robotkar ide-oda mozog 2 másodpercenként.

---

## Összefoglalás

```
UR fogalom          →   amit csináltunk
─────────────────────────────────────────
Waypoint A          →   poz_a = [0.0, 0.0]
Waypoint B          →   poz_b = [1.57, -0.785]
MoveJ               →   Float64MultiArray küldése
Program loop        →   create_timer(2.0, callback)
Joint state         →   /joint_states topic
```

---

## Következő lépés

[04_moveit2_alapok.md](04_moveit2_alapok.md) – MoveIt2: automatikus pályatervezés, ütközéskerülés
