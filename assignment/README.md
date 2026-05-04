# Vizuális Visszacsatoláson Alapuló Robotkar Vezérlőrendszer
## BSc Szakdolgozat – Rendszerterv

**Szerző:** Ujj Norbert (NLUG4F)
**Intézmény:** Óbudai Egyetem – Neumann János Informatikai Kar

---

## 1. Projekt Összefoglaló

### Mi a feladat?
Egy robotkar vezérlőrendszerének megtervezése, amely kamera képe alapján képes egy fali csatlakozót (vagy hasonló célelemet) pontosan azonosítani, és a robotkar végén tartott csatlakozódugaszt precízen beilleszteni – előre ismert koordináták nélkül.

A rendszer az **Image-Based Visual Servoing (IBVS)** módszert alkalmazza perspektív transzformációval, a Cao (2022) által leírt megközelítés alapján.

### Mi TARTOZIK a rendszerhez
- 1 robotkar (3+ szabadságfok)
- 1 fix kamera (eye-to-hand elrendezés)
- Vizuális feldolgozó pipeline
- IBVS vezérlő perspektív transzformációval
- Szimulációs környezet (ROS2 + Gazebo)

### Mi NEM tartozik a rendszerhez
- Autonóm navigáció / mobilitás
- Digitális tervrajz feldolgozás
- Szerszámcsere mechanizmus
- Több robotkar kezelése

---

## 2. Rendszer Komponensei

### 2.1 Kamera alrendszer
- **Típus:** Fix elhelyezésű RGB kamera (eye-to-hand konfiguráció)
- **Pozíció:** A fal közelében, oldalirányból látja mind a csatlakozót, mind a robotkar végpontját
- **Feladat:** Valós idejű képfolyam biztosítása a vizuális feldolgozónak
- **Szimuláció:** Gazebo kamera plugin (sensor_msgs/Image topic-on)
- **Labor:** USB webcam (pl. Logitech C920) vagy ipari kamera

### 2.2 Robot kar
- **Szabadságfokok:** minimum 3 DOF (X, Y, Z mozgás + orientáció)
- **Szimulált modell:** Universal Robots UR3/UR5 URDF modell (vagy hasonló)
- **Vezérlési interfész:** ROS2 action server (FollowJointTrajectory)
- **Labor:** Az egyetemi robotikai laborban elérhető kar

### 2.3 End-effector (megfogó)
- **Feladat:** A csatlakozódugaszt stabilan tartja az illesztés során
- **Típus:** Egyszerű párhuzamos fogó (parallel gripper)
- **Szimuláció:** Fix csatlakozó a kar végére modellezve

### 2.4 Vezérlő számítógép
- **OS:** Ubuntu 22.04 (ROS2 Humble)
- **Feladatok:** Képfeldolgozás, vezérlőjel számítás, robot kommunikáció
- **Teljesítmény:** Standard laptop/PC elegendő szimulációhoz

---

## 3. Szoftver Architektúra

### 3.1 Keretrendszer
**ROS2 (Humble)** – middleware, amely kezeli a komponensek közötti kommunikációt.

### 3.2 Node-ok

```
┌─────────────────┐     /image_raw      ┌─────────────────┐
│  camera_node    │ ──────────────────> │  vision_node    │
│                 │                     │                 │
│ Kamera képet   │                     │ Képfeldolgozás  │
│ publikálja      │                     │ Jellemzőkinyerés│
└─────────────────┘                     │ Hiba vektor     │
                                        └────────┬────────┘
                                                 │ /visual_error
                                                 ▼
┌─────────────────┐   /joint_trajectory  ┌─────────────────┐
│  robot_node     │ <──────────────────  │ controller_node │
│                 │                     │                 │
│ Robot kar       │                     │ IBVS vezérlő    │
│ mozgatása       │                     │ Perspektív transz│
└─────────────────┘                     └─────────────────┘
```

### 3.3 Topic-ok és Service-ek

| Topic / Service | Típus | Leírás |
|---|---|---|
| `/image_raw` | sensor_msgs/Image | Nyers kameraképek |
| `/camera_info` | sensor_msgs/CameraInfo | Kamera kalibrációs adatok |
| `/visual_error` | geometry_msgs/Twist | Képtérbeli hiba vektor |
| `/joint_trajectory` | trajectory_msgs/JointTrajectory | Karmozgás parancs |
| `/robot_state` | sensor_msgs/JointState | Kar aktuális állapota |
| `/assembly_status` | std_msgs/String | Illesztés státusza |

### 3.4 Technológiai stack

| Komponens | Technológia |
|---|---|
| Middleware | ROS2 Humble |
| Szimuláció | Gazebo Fortress |
| Képfeldolgozás | OpenCV 4.x |
| Programozási nyelv | Python 3.10 |
| Vizualizáció | RViz2 |
| Robotmodell | URDF / xacro |

---

## 4. Vision Pipeline

A vizuális feldolgozás lépései a kamera képtől a hibavektórig:

```
Kamera kép (RGB)
        │
        ▼
┌───────────────────┐
│ Előfeldolgozás    │  - Szürkeárnyalatos konverzió
│                   │  - Zajszűrés (Gaussian blur)
│                   │  - Kontraszt normalizálás
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ Jellemzőkinyerés  │  - Konnektor keretet/lyukakat detektálja
│                   │  - Éldetektor (Canny)
│                   │  - Kontúr keresés
│                   │  - Ellipszis/téglalap illesztés
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ Homográfia        │  - DLT módszerrel kalibrált H mátrix
│ transzformáció    │  - Célpont és aktuális pozíció vetítése
│                   │    virtuális képsíkra
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ Hiba számítás     │  - e = f_kívánt - f_aktuális
│                   │  - X, Y irányú eltérés képtérben
│                   │  - Z irányú eltérés (mélység)
└────────┬──────────┘
         │
         ▼
   Hiba vektor → controller_node
```

### 4.1 Kamera kalibráció
- **Módszer:** Direct Linear Transformation (DLT)
- **Kalibrációs objektum:** Sakktábla minta (9x6 belső sarok)
- **Eredmény:** Intrinsic mátrix (K), torzítási együtthatók, homográfia mátrixok

### 4.2 Célpont azonosítása
A konnektor (célpont) vizuális jellemzői alapján azonosítható:
- Téglalap alakú keret (éldetektor)
- A csatlakozó lyukainak körvonalai
- Esetleges segédjelölők (ha szükséges a megbízhatósághoz)

---

## 5. Vezérlő Rendszer (IBVS)

### 5.1 Működési elv
A perspektív transzformáción alapuló IBVS vezérlő (Cao 2022 alapján):

1. **Virtuális képsík létrehozása** – A kalibráció során meghatározott H mátrixokkal
2. **Objektumok vetítése** – Célpont és aktuális pozíció a virtuális síkra vetítve
3. **Hiba számítás** – `e = s* - s` ahol `s*` a kívánt, `s` az aktuális képi jellemző
4. **Vezérlőjel** – `v = -λ * L⁺ * e` ahol L az interakciós mátrix

### 5.2 Fázisok

**X/Y igazítás (képtérbeli):**
- A vezérlő addig mozgatja a kart, amíg a csatlakozó képi pozíciója egybeesik a célpontéval
- Ez a perspektív transzformáción alapuló IBVS fő alkalmazása

**Z irányú közelítés (mélység):**
- A képjellemző méretének változása alapján
- Vagy: a kamera felülnézeti helyzetbe mozog → mélység meghatározás → visszaáll

**Dokkolás (illesztés):**
- Lineáris pályán, az orientáció megtartásával
- Attitűd-kivonásos módszer (attitude extraction)

### 5.3 ADRC vezérlő
- **Cél:** Zajok és nem modellezett zavarok kompenzálása
- **Komponensek:** Kiterjesztett Állapotmegfigyelő (ESO) + nemlineáris állapotvisszacsatolás
- **Előny:** Robusztus, nem igényel precíz rendszermodellt

---

## 6. Szimulációs Környezet

### 6.1 Gazebo world
```
gazebo_world/
├── wall.sdf          - Fal modell
├── connector.sdf     - Fali csatlakozó (célpont)
└── environment.world - Teljes környezet
```

### 6.2 Robot modell
```
robot_description/
├── urdf/
│   ├── robot_arm.urdf.xacro   - Robot kar leírás
│   └── camera_mount.xacro     - Kamera tartó
└── config/
    └── joint_limits.yaml      - Ízület korlátok
```

### 6.3 ROS2 csomagstruktúra
```
visual_servoing_ws/
├── src/
│   ├── camera_node/           - Kamera illesztő node
│   ├── vision_node/           - Képfeldolgozás
│   ├── controller_node/       - IBVS vezérlő
│   ├── robot_node/            - Robot interfész
│   └── robot_description/     - URDF modellek
├── launch/
│   ├── simulation.launch.py   - Gazebo szimuláció indítása
│   └── real_robot.launch.py   - Valódi hardware indítása
└── config/
    └── params.yaml            - Rendszer paraméterek
```

---

## 7. Fejlesztési Fázisok

### 7.1 Fázis – Szimuláció (ROS2 + Gazebo)
- [ ] Szimulációs környezet felépítése (fal + konnektor modell)
- [ ] Robot kar URDF modell integrálása
- [ ] Kamera plugin konfigurálása
- [ ] Vision pipeline implementálása (OpenCV)
- [ ] IBVS vezérlő implementálása
- [ ] Integrációs tesztelés szimulációban

### 7.2 Fázis – Valódi Hardware (Egyetemi labor)
- [ ] Kamera kalibráció valódi kamerával
- [ ] Robot interfész adaptálása (lab hardware)
- [ ] Finomhangolás valódi körülmények között
- [ ] Teljesítménymérés és dokumentálás

---

## 8. Rendszerterv Dokumentum Struktúra

A szakdolgozat rendszerterv fejezete az alábbi részekből áll:

1. **Rendszer áttekintés** – Célok, scope, architektúra diagram
2. **Hardware tervezés** – Komponens specifikációk, elrendezés
3. **Szoftver architektúra** – Node-ok, interfészek, adatfolyam
4. **Vision alrendszer tervezése** – Pipeline, kalibráció, jellemzők
5. **Vezérlő tervezése** – IBVS matematika, ADRC, pályatervezés
6. **Szimulációs környezet** – Gazebo setup, tesztkörnyezet
7. **Tesztelési terv** – Metrikák, tesztesetek, sikerkritériumok
8. **Kockázatelemzés** – Technikai kihívások és megoldásaik

---

## Irodalom

- [1] Cao C. *Research on a Visual Servoing Control Method Based on Perspective Transformation under Spatial Constraint.* Machines. 2022; 10(11):1090.
- [2] Pomares J. *Visual Servoing in Robotics.* Electronics. 2019; 8(11):1298.
- [3] Yan S, Tao X, Xu D. *High-precision robotic assembly system using three-dimensional vision.* Int. Journal of Advanced Robotic Systems. 2021;18(3).
