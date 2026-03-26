# 02 – Gazebo: Első lépések

**Előfeltétel:** [01_ros2_alapok.md](01_ros2_alapok.md) kész

---

## Mi a Gazebo?

A Gazebo egy fizikai szimulációs program – olyan mint egy "virtuális labor".

**Analógia:** Az UR-nél a PolyScope szimulátora mutatja a robot mozgását. A Gazebo ezt teszi, de sokkal több eszközzel: fizika, gravitáció, kamerák, szenzorok, ütközések.

```
UR világ:              ROS2 + Gazebo:
┌─────────────┐        ┌───────────┐     ┌──────────┐
│ PolyScope   │        │  ROS2     │────▶│  Gazebo  │
│ Szimulátor  │   →    │  Node-ok  │◀────│ (fizika) │
└─────────────┘        └───────────┘     └──────────┘
```

---

## 1. Gazebo elindítása

```bash
# Egyszerű üres világ
gz sim

# Vagy egy előre definiált világgal:
gz sim shapes.sdf
```

Megnyílik egy ablak. Navigáció:
- **Bal egér + húzás** = forgatás
- **Középső egér / scroll** = zoom
- **Jobb egér + húzás** = mozgatás
- **Play gomb** (bal alsó) = szimuláció indítása/megállítása

---

## 2. Modell hozzáadása az UI-ból

1. Gazebo ablakban: bal oldali panelen keresd az **Insert** gombot (vagy `Ctrl+M`)
2. Online modellek közül válassz egyet (pl. "Simple Box")
3. Kattints a világ egy pontjára – ott jelenik meg

**Megjegyzés:** Az online modellek letöltéséhez internet kell. Első alkalommal lassú lehet.

---

## 3. Saját világ SDF fájlban

Az SDF (Simulation Description Format) egy XML alapú fájlformátum.

Hozd létre:

```bash
mkdir -p ~/gazebo_tanulas
nano ~/gazebo_tanulas/elso_vilag.sdf
```

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="elso_vilag">

    <!-- Alapértelmezett fizika (ODE) -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Fény -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Talaj -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane>
            <normal>0 0 1</normal>
            <size>100 100</size>
          </plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane>
            <normal>0 0 1</normal>
            <size>100 100</size>
          </plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Egy egyszerű doboz -->
    <model name="piros_doboz">
      <pose>2 0 0.5  0 0 0</pose>   <!-- x=2m, z=0.5m (a talajon) -->
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx><iyy>0.083</iyy><izz>0.083</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 1 1</size></box></geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>   <!-- piros -->
            <diffuse>0.8 0.1 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

Indítás:
```bash
gz sim ~/gazebo_tanulas/elso_vilag.sdf
```

Látni fogsz egy piros dobozt. Nyomd meg a Play gombot – a doboz leesik (gravitáció).

---

## 4. A pose formátuma

A `<pose>` mindig 6 szám:
```
x  y  z  roll  pitch  yaw
```

- **x, y, z**: pozíció méterben
- **roll, pitch, yaw**: forgatás radiánban (0 = nem forgat)

Példák:
```xml
<pose>0 0 0 0 0 0</pose>        <!-- origó, nincs forgatás -->
<pose>1 0 0.5 0 0 0</pose>      <!-- 1m előre, 0.5m fel -->
<pose>0 0 0 0 0 1.5708</pose>   <!-- 90 fok elforgatva (π/2 radián) -->
```

---

## 5. Statikus vs dinamikus objektum

```xml
<!-- Statikus: nem mozog, nem esik le (pl. fal, talaj) -->
<model name="fal">
  <static>true</static>
  ...
</model>

<!-- Dinamikus: van tömege, esik, ütközik (alapértelmezett) -->
<model name="labda">
  <!-- nincs <static> tag -->
  <link name="link">
    <inertial><mass>0.5</mass>...</inertial>
    ...
  </link>
</model>
```

---

## 6. Gazebo + ROS2 összekötés

A `ros_gz_bridge` csomag fordítja le a Gazebo üzeneteket ROS2 üzenetekké.

```bash
# Terminál 1: Gazebo
gz sim ~/gazebo_tanulas/elso_vilag.sdf

# Terminál 2: Bridge indítása
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/elso_vilag/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

# Terminál 3: Olvasd a szimulációs időt
ros2 topic echo /world/elso_vilag/clock
```

---

## Összefoglalás

```
amit megtanultál:
├── Gazebo megnyitása, navigáció az UI-ban
├── SDF fájl struktúra (world → model → link → collision/visual)
├── pose formátuma: x y z roll pitch yaw
├── static vs dinamikus objektumok
└── Gazebo + ROS2 összekötés (bridge)
```

---

## Következő lépés

[03_robot_hello_world.md](03_robot_hello_world.md)
