# 01 – ROS2 Alapok: Node, Topic, Publisher, Subscriber

**Előfeltétel:** [00_telepites.md](00_telepites.md) kész

---

## Mi a ROS2?

A ROS2 (Robot Operating System 2) nem egy operációs rendszer, hanem egy **middleware** – egy kommunikációs réteg programok között.

**Analógia az UR robothoz:**
- UR-nél a PolyScope UI fogja össze a programokat
- ROS2-ben a DDS (Data Distribution Service) kommunikációs réteg fogja össze a node-okat

```
UR világ:          ROS2 világ:
┌──────────┐       ┌──────────┐   topic    ┌──────────┐
│PolyScope │  →    │  Node A  │ ──────────→ │  Node B  │
│  (UI)    │       │(publisher)│            │(subscriber│
└──────────┘       └──────────┘            └──────────┘
```

---

## 1. Az első node – parancssorból

Nyiss egy terminált és próbáld ki:

```bash
# Indíts egy "talker" node-ot (adatot küld)
ros2 run demo_nodes_cpp talker
```

Másik terminálban:
```bash
# Nézd meg, milyen topic-ok vannak
ros2 topic list
# Látod: /chatter

# Olvasd ki a topic tartalmát
ros2 topic echo /chatter
```

**Mit látsz:** `data: 'Hello World: 5'` – a talker másodpercenként küld.

Állítsd le: `Ctrl+C`

---

## 2. Saját package létrehozása

```bash
# Hozz létre egy munkamappát
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Csomag létrehozása (Python)
ros2 pkg create --build-type ament_python tanulas_pkg

# Mappastruktúra amit kaptál:
# tanulas_pkg/
# ├── package.xml         ← metaadat
# ├── setup.py            ← Python setup
# ├── resource/
# └── tanulas_pkg/
#     └── __init__.py
```

---

## 3. Első saját Publisher node

Hozd létre a fájlt:

```bash
nano ~/ros2_ws/src/tanulas_pkg/tanulas_pkg/publisher_node.py
```

Másold be:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')          # node neve
        self.publisher_ = self.create_publisher(
            String,        # üzenet típus
            'sajat_topic', # topic neve
            10             # queue méret
        )
        # Timer: minden 1 másodpercben hív egy függvényt
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Szia ROS2! Szam: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Kuldtem: {msg.data}')
        self.counter += 1

def main():
    rclpy.init()
    node = PublisherNode()
    rclpy.spin(node)   # "pörög" amíg Ctrl+C-t nem nyomsz
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 4. Első saját Subscriber node

```bash
nano ~/ros2_ws/src/tanulas_pkg/tanulas_pkg/subscriber_node.py
```

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'sajat_topic',   # ugyanaz a topic neve!
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Kaptam: {msg.data}')

def main():
    rclpy.init()
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 5. setup.py módosítása

A node-ok futtathatóvá tételéhez:

```bash
nano ~/ros2_ws/src/tanulas_pkg/setup.py
```

Az `entry_points` részt módosítsd:

```python
entry_points={
    'console_scripts': [
        'publisher = tanulas_pkg.publisher_node:main',
        'subscriber = tanulas_pkg.subscriber_node:main',
    ],
},
```

---

## 6. Build és futtatás

```bash
cd ~/ros2_ws
colcon build --packages-select tanulas_pkg

# Betöltés (minden új terminálban szükséges!)
source ~/ros2_ws/install/setup.bash
```

Terminál 1:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run tanulas_pkg publisher
```

Terminál 2:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run tanulas_pkg subscriber
```

**Eredmény:** A subscriber kiírja amit a publisher küld.

---

## 7. Hasznos parancsok

```bash
# Milyen node-ok futnak?
ros2 node list

# Milyen topic-ok vannak?
ros2 topic list

# Részletek egy topicról
ros2 topic info /sajat_topic

# Kiolvasni egy topic értékét
ros2 topic echo /sajat_topic

# Mennyi üzenetet küld másodpercenként?
ros2 topic hz /sajat_topic

# Manuálisan küldeni egy topic-ra (teszteléshez)
ros2 topic pub /sajat_topic std_msgs/msg/String "data: 'Teszt uzenet'"
```

---

## Összefoglalás

```
amit megtanultál:
├── Node = önálló Python program, ami ROS2-t használ
├── Publisher = küld adatot egy topic-ra
├── Subscriber = fogad adatot egy topic-ról
├── Topic = névvel ellátott kommunikációs csatorna
└── colcon build = lefordítja és telepíti a package-et
```

---

## Következő lépés

[02_gazebo_elso_lepesek.md](02_gazebo_elso_lepesek.md)
