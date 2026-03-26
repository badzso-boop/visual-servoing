# ROS2 + Gazebo Tanulási Terv

**Cél:** Eljutni nulláról a vizuális szervo projekt megértéséig.

**Rendszer:** Ubuntu 24.04 WSL2 + Windows 11 (WSLg GUI)

---

## Tanulási Útvonal

| # | Lecke | Leírás | Státusz |
|---|-------|--------|---------|
| 0 | [Telepítés](00_telepites.md) | ROS2 Jazzy + Gazebo Harmonic telepítése | ⬜ |
| 1 | [ROS2 alapok](01_ros2_alapok.md) | Node, Topic, Publisher, Subscriber | ⬜ |
| 2 | [Gazebo első lépések](02_gazebo_elso_lepesek.md) | Üres világ, modellek elhelyezése | ⬜ |
| 3 | [Robot Hello World](03_robot_hello_world.md) | Robot betöltése, joint mozgatása | ⬜ |
| 4 | [MoveIt2 alapok](04_moveit2_alapok.md) | Waypoint, lineáris mozgás, tervezés | ⬜ |
| 5 | [Kamera ROS2-ben](05_kamera.md) | Szimulált kamera, képstream fogadása | ⬜ |

**Ajánlott sorrend:** Sorban haladj, ne ugorj előre.

---

## Előzetes tudásod (UR robotokból)

Amit már tudsz, és hogyan kapcsolódik ROS2-höz:

| UR UI fogalom | ROS2 megfelelője |
|---------------|------------------|
| Waypoint | Célpont (`geometry_msgs/Pose`) |
| MoveJ (joint mozgás) | `moveit` joint-tér tervező |
| MoveL (lineáris) | `moveit` Descartes/lineáris tervező |
| Digital I/O | ROS2 service / topic |
| Program futtatása | Node indítása (`ros2 run`) |
| PolyScope UI | RViz2 vizualizáció |

---

## Fontos fogalmak (előzetes olvasmány)

- **Node** = egy önálló program (pl. "kamera olvasó", "vezérlő")
- **Topic** = névvel ellátott csatorna, amin adatot lehet küldeni/fogadni
- **Publisher** = aki adatot küld egy topicra
- **Subscriber** = aki adatot fogad egy topicról
- **Package** = összefüggő node-ok és fájlok csomagja
- **Launch file** = több node egyszerre indítása
- **URDF** = robot leíró fájl (kinematika, megjelenés)
