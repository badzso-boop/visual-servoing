# Vizuális Visszacsatoláson Alapuló Robotkar Vezérlőrendszer
## Részletes Rendszerterv

**Szerző:** Ujj Norbert (NLUG4F)
**Intézmény:** Óbudai Egyetem – Neumann János Informatikai Kar
**Verzió:** 1.0

---

# Tartalomjegyzék

1. [Rendszer Áttekintés](#1-rendszer-áttekintés)
2. [Hardware Specifikáció](#2-hardware-specifikáció)
3. [Szoftver Architektúra](#3-szoftver-architektúra)
4. [Vision Alrendszer](#4-vision-alrendszer)
5. [IBVS Vezérlő – Matematikai Alapok](#5-ibvs-vezérlő--matematikai-alapok)
6. [ADRC Vezérlő](#6-adrc-vezérlő)
7. [Szimulációs Környezet](#7-szimulációs-környezet)
8. [Tesztelési Terv](#8-tesztelési-terv)
9. [Kockázatelemzés](#9-kockázatelemzés)
10. [Irodalom és Hivatkozások](#10-irodalom-és-hivatkozások)

---

# 1. Rendszer Áttekintés

## 1.1 A feladat definíciója

A rendszer célja egy robotkar automatikus, vizuális visszacsatoláson alapuló vezérlése, amelynek feladata egy fali csatlakozóaljzatba (vagy általánosítva: egy előre definiált lyukba) egy csatlakozódugaszt pontosan beilleszteni. A robot nem rendelkezik előre programozott koordinátákkal a célpont helyzetéről – a pozicionálás kizárólag a kamera által közvetített képi információ alapján történik valós időben.

Ez a megközelítés az **Image-Based Visual Servoing (IBVS)** módszercsaládon alapul, amelyet Cao (2022) perspektív transzformációval egészít ki a térbeli korlátok kezelésére. Az irodalomkutatás során feltárt eredmények azt mutatják, hogy ez a módszer kevesebb mint 1 mm-es pozícionálási pontossággal, 100%-os sikerességi aránnyal képes működni valós környezetben is.

## 1.2 Rendszerhatárok (Scope)

**A rendszer részét képezi:**
- Egy robotkar és a hozzá tartozó vezérlőrendszer
- Egy fix elhelyezésű kamera (eye-to-hand konfiguráció)
- A teljes vizuális feldolgozási pipeline
- Az IBVS + ADRC vezérlő
- ROS2 alapú szoftver architektúra
- Gazebo szimulációs környezet

**A rendszer NEM tartalmazza:**
- Autonóm navigációt (a robot platformja manuálisan kerül a fal elé)
- Szerszámcserét (a csatlakozódugasz fix az end-effectoron)
- Digitális tervrajz feldolgozást
- Több robotkar kezelését

## 1.3 Rendszer architektúra – Áttekintő diagram

```
╔══════════════════════════════════════════════════════════════╗
║                    FIZIKAI KÖRNYEZET                         ║
║                                                              ║
║   [FALI CSATLAKOZÓ]          [ROBOTKAR]                     ║
║        (célpont)           [CSATLAKOZÓDUGASZ]               ║
║            │                      │                          ║
║            └──────────┬───────────┘                          ║
║                       │                                      ║
║                  [KAMERA]  ← fix, eye-to-hand                ║
╚══════════════════════════════════════════════════════════════╝
                       │
                  képfolyam
                       │
╔══════════════════════▼═══════════════════════════════════════╗
║                  SZOFTVER RÉTEG (ROS2)                       ║
║                                                              ║
║  ┌─────────────┐    ┌─────────────┐    ┌─────────────────┐  ║
║  │ camera_node │───>│ vision_node │───>│ controller_node │  ║
║  │             │    │             │    │                 │  ║
║  │ Képfolyam   │    │ Jellemző-   │    │ IBVS + ADRC     │  ║
║  │ kezelése    │    │ kinyerés    │    │ vezérlő         │  ║
║  │ Kalibráció  │    │ Homográfia  │    │ Pályatervezés   │  ║
║  └─────────────┘    └─────────────┘    └────────┬────────┘  ║
║                                                 │            ║
║                                        mozgásparancs         ║
║                                                 │            ║
║                                        ┌────────▼────────┐  ║
║                                        │   robot_node    │  ║
║                                        │                 │  ║
║                                        │ Kar interfész   │  ║
║                                        │ Ízületvezérlés  │  ║
║                                        └────────┬────────┘  ║
╚═════════════════════════════════════════════════╪════════════╝
                                                  │
                                         ízületparancsok
                                                  │
                                         ┌────────▼────────┐
                                         │   ROBOT DRIVER  │
                                         │ (UR ROS2 driver │
                                         │  vagy Gazebo)   │
                                         └─────────────────┘
```

## 1.4 Működési folyamat – Lépésről lépésre

A rendszer az alábbi sorrendben hajtja végre az illesztési feladatot:

**1. Inicializáció**
A robot egy előre definiált "várakozási" pozícióban áll, a csatlakozódugasz a kar végén rögzítve. A kamera aktív, képfolyam elérhető.

**2. Célpont azonosítás**
A vision_node elemzi a kamera képét, és megkísérli azonosítani a fali csatlakozóaljzatot annak vizuális jellemzői (keret, lyukak körvonalai) alapján.

**3. Hibavektor számítás**
A rendszer kiszámítja az aktuális képi pozíció és a kívánt (illesztési) pozíció közötti eltérést a virtuális képsíkon.

**4. X/Y igazítás**
Az IBVS vezérlő mozgatja a kart, amíg a dugasz képi pozíciója egybeesik az aljzat képi pozíciójával a vízszintes és függőleges tengelyeken.

**5. Z irányú közelítés**
A mélységi eltérés kompenzálása után a kar lassan előre mozdul, a kamera folyamatosan ellenőrzi az igazítást.

**6. Dokkolás**
Lineáris, egyenes vonalú mozgással az end-effector behelyezi a csatlakozódugaszt az aljzatba.

**7. Visszajelzés**
A rendszer érzékeli az illesztés sikerét (erőszenzor vagy vizuális visszajelzés alapján) és visszatér a várakozási pozícióba.

---

# 2. Hardware Specifikáció

## 2.1 Robot kar

### Javasolt típus: Universal Robots UR3e

A UR3e egy 6 szabadságfokú kollaboratív robot (cobot), amely széles körben elterjedt egyetemi laboratóriumokban és kutatóintézetekben. Főbb jellemzői:

| Tulajdonság | Érték |
|---|---|
| Szabadságfokok | 6 DOF |
| Kinyúlás (reach) | 500 mm |
| Terhelhetőség | 3 kg |
| Ismétlési pontosság | ±0.03 mm |
| Tömeg | 11.2 kg |
| Kommunikáció | Ethernet (TCP/IP, RTDE protokoll) |
| ROS2 driver | universal_robots_ros2_driver |

A UR3e előnye, hogy rendelkezik egy beépített erő/nyomaték szenzorral a csuklóban (Force/Torque sensor), amely az illesztési folyamat során érzékeli, ha mechanikai akadály van – ez biztonságossá teszi a dokkolási fázist.

**Hivatalos ROS2 driver dokumentáció:**
`https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver`

A driver az RTDE (Real-Time Data Exchange) protokollon keresztül kommunikál a robot vezérlőjével 125 Hz-es frissítési rátával.

### Alternatíva: myCobot 280

Ha az egyetemi laborban nem UR robot áll rendelkezésre, a myCobot 280 (Elephant Robotics) egy olcsóbb alternatíva:

| Tulajdonság | Érték |
|---|---|
| Szabadságfokok | 6 DOF |
| Kinyúlás | 280 mm |
| Terhelhetőség | 250 g |
| Ismétlési pontosság | ±0.5 mm |
| Kommunikáció | USB / Serial |
| ROS2 támogatás | mycobot_ros2 csomag |

## 2.2 End-effector (megfogó)

Az end-effector feladata a csatlakozódugasz stabil tartása az illesztési folyamat során. Két tervezési megközelítés létezik:

### Opció A: Egyszerű párhuzamos fogó (párhuzamos ujjas megfogó)

Ez egy standard megfogó, amelyet sokan alkalmaznak ipari és kutatási környezetben. Az ujjak párhuzamosan mozognak, és stabil fogást biztosítanak hengerszimmetrikus tárgyakra.

- **Javasolt típus:** Robotiq 2F-85
- **Nyílásszélesség:** 0–85 mm
- **Erő:** 20–235 N
- **ROS2 driver:** `robotiq_ros2` csomag

### Opció B: Egyedi 3D nyomtatott tartó

Szimulációhoz és prototípushoz egy egyszerűbb megoldás: a csatlakozódugasz egy 3D nyomtatott tartóba van rögzítve, amely az adapter nélkül közvetlenül a kar végére szerelhető. Ez nem igényel külön vezérlést.

## 2.3 Kamera

### Javasolt típus: Intel RealSense D435i

| Tulajdonság | Érték |
|---|---|
| Típus | RGB-D (színes + mélységi) |
| RGB felbontás | max 1920×1080 @ 30fps |
| Mélység felbontás | max 1280×720 @ 30fps |
| Látószög (RGB) | 69° × 42° |
| Mélységi tartomány | 0.1 – 10 m |
| Interfész | USB 3.0 |
| ROS2 driver | `realsense2_camera` csomag |

**Dokumentáció:** `https://github.com/IntelRealSense/realsense-ros`

A mélységi csatorna az 1. fázisban nem feltétlenül szükséges – a Z irányú pozicionálás a képjellemzők méretváltozásából is levezethető. Azonban ha rendelkezésre áll, jelentősen egyszerűsíti és megbízhatóbbá teszi a mélységbecslést.

### Alternatíva: Logitech C920 (csak RGB)

Ha mélységi kamera nem szükséges vagy nem elérhető:

| Tulajdonság | Érték |
|---|---|
| Típus | RGB webcam |
| Felbontás | max 1920×1080 @ 30fps |
| Látószög | 78° |
| Interfész | USB 2.0 |
| ROS2 driver | `v4l2_camera` csomag |

### Kamera elhelyezése (eye-to-hand konfiguráció)

```
         [KAMERA]
           /│\
          / │ \  ← látómező
         /  │  \
        /   │   \
[DUGASZ]   [KAR] [ALJZAT]
   (kar     (fal)
   végén)
```

A kamerát a faltól kb. 0.5–1 m távolságra, oldalirányban kell elhelyezni, úgy hogy látja:
1. A fali csatlakozóaljzatot (célpont)
2. A robotkar végpontját és az ott tartott dugaszt (aktuális pozíció)

A kamera optikai tengelye kb. 45°-os szögben néz a fal felé. Ez biztosítja, hogy mind az X/Y síkbeli eltérés, mind a Z irányú közelítés jól megfigyelhető legyen.

## 2.4 Vezérlő számítógép

| Tulajdonság | Minimális | Javasolt |
|---|---|---|
| CPU | Intel i5 (8. gen) | Intel i7 (11. gen) |
| RAM | 8 GB | 16 GB |
| GPU | Nincs szükség | Nincs szükség |
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |
| ROS verzió | ROS2 Humble | ROS2 Humble |

A GPU nem kritikus, mivel a képfeldolgozás OpenCV-vel CPU-n is real-time módon futtatható 1080p képen ~30fps-sel.

---

# 3. Szoftver Architektúra

## 3.1 ROS2 áttekintés

A **Robot Operating System 2 (ROS2)** egy nyílt forráskódú middleware keretrendszer robotikai alkalmazásokhoz. Főbb tulajdonságai:

- **Publish-Subscribe kommunikáció:** A node-ok topic-okon keresztül üzeneteket küldenek egymásnak, anélkül hogy közvetlen kapcsolatot tartanának fenn. Ez lazán csatolt, moduláris architektúrát eredményez.
- **DDS (Data Distribution Service):** Az ROS2 DDS-t használ az üzenetek továbbítására, amely valós idejű, determinisztikus kommunikációt biztosít.
- **Action-ok és Service-ek:** Hosszabb futású feladatokhoz (pl. robot mozgás) action-ök, szinkron lekérdezésekhez service-ek használatosak.

**Hivatalos dokumentáció:** `https://docs.ros.org/en/humble/`

A rendszerhez az **ROS2 Humble Hawksbill** verziót választjuk, amely az Ubuntu 22.04 LTS-sel kompatibilis, és 2027-ig kap biztonsági frissítéseket (Long Term Support).

## 3.2 Node-ok részletes leírása

### 3.2.1 camera_node

**Feladata:** A fizikai kamera (vagy szimulált kamera) képfolyamának kezelése és publikálása ROS2 topic-ra.

**Bemenetek:**
- Fizikai kamera: USB kapcsolaton érkező képfolyam
- Szimulációban: Gazebo `/camera/image_raw` topic

**Kimenetek:**

| Topic | Típus | Leírás |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Nyers RGB képkockák (30 fps) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Kalibrációs mátrixok, torzítási együtthatók |

**Konfiguráció:**
```yaml
# config/camera_params.yaml
camera_node:
  ros__parameters:
    image_width: 1280
    image_height: 720
    fps: 30
    camera_frame: "camera_optical_frame"
    calibration_file: "config/camera_calibration.yaml"
```

A `camera_info` topic-on közzétett adatok tartalmazzák a kamera intrinsic paramétereit (K mátrix), amelyeket a kalibráció során határozunk meg (lásd 4.1 fejezet).

### 3.2.2 vision_node

**Feladata:** A nyers képből kinyerni a releváns képi jellemzőket, kiszámítani a hibavektort, és azt közzétenni a controller számára.

**Bemenetek:**

| Topic | Típus |
|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` |

**Kimenetek:**

| Topic | Típus | Leírás |
|---|---|---|
| `/vision/visual_error` | `geometry_msgs/Twist` | Képtérbeli hibavektor (x, y, z komponensek) |
| `/vision/debug_image` | `sensor_msgs/Image` | Annotált kép (debuggoláshoz, RViz2-ben megjeleníthető) |
| `/vision/target_detected` | `std_msgs/Bool` | Igaz, ha a célpont látható |

**Belső állapot:**
- Tárolja a homográfia mátrixokat (H_a, H_b)
- Tárolja a kívánt képi jellemzőket (kalibrációs fázisból)
- Nyilvántartja a detektálás stabilitását (hány egymást követő képkockán volt sikeres)

**Pszeudokód – fő hurok:**

```
FÜGGVÉNY kepkocka_feldolgozo(kep):
    # Előfeldolgozás
    szurke_kep = szurke_konverzio(kep)
    simított_kep = gaussian_szures(szurke_kep, kernel=5)

    # Célpont detektálás
    aljzat_jellemzok = aljzat_detektalo(simított_kep)
    HA aljzat_jellemzok ÜRES:
        celpoint_latható = HAMIS
        RETURN

    # Dugasz pozíció detektálás
    dugasz_jellemzok = dugasz_detektalo(simított_kep)

    # Homográfia alkalmazása – virtuális képsíkra vetítés
    aljzat_virtualis = homografia_alkalmaz(H_a, aljzat_jellemzok)
    dugasz_virtualis = homografia_alkalmaz(H_b, dugasz_jellemzok)

    # Hibavektor számítás
    hiba_x = aljzat_virtualis.x - dugasz_virtualis.x
    hiba_y = aljzat_virtualis.y - dugasz_virtualis.y
    hiba_z = melyseq_becsles(aljzat_jellemzok)

    # Publikálás
    hiba_uzenet = Twist()
    hiba_uzenet.linear.x = hiba_x
    hiba_uzenet.linear.y = hiba_y
    hiba_uzenet.linear.z = hiba_z
    visual_error_publisher.publish(hiba_uzenet)
```

### 3.2.3 controller_node

**Feladata:** A hibavektor alapján kiszámítani a szükséges robot mozgásparancsot az IBVS + ADRC algoritmus alapján.

**Bemenetek:**

| Topic | Típus |
|---|---|
| `/vision/visual_error` | `geometry_msgs/Twist` |
| `/vision/target_detected` | `std_msgs/Bool` |
| `/robot/joint_states` | `sensor_msgs/JointState` |

**Kimenetek:**

| Topic / Action | Típus | Leírás |
|---|---|---|
| `/robot/cartesian_velocity` | `geometry_msgs/Twist` | Descartes-térbeli sebesség parancs |
| `/assembly/status` | `std_msgs/String` | Illesztés státusza |

**Állapotgép:**

A vezérlő egy állapotgépen (state machine) alapul, amely az illesztési folyamat különböző fázisait kezeli:

```
VÁRAKOZÁS
    │
    │ célpont detektálva
    ▼
XY_IGAZÍTÁS ──── hiba > küszöb ──→ (marad XY_IGAZÍTÁS)
    │
    │ xy_hiba < 5 pixel
    ▼
Z_KÖZELÍTÉS ──── nem elég közel ──→ (marad Z_KÖZELÍTÉS)
    │
    │ z_hiba < küszöb
    ▼
DOKKOL ──────── erő > limit ──────→ HIBA_KEZELÉS
    │
    │ sikeres illesztés
    ▼
KÉSZ
```

**Pszeudokód – vezérlő hurok:**

```
FÜGGVÉNY vezerlo_lepes(hiba_vektor, allapot):
    HA allapot == XY_IGAZÍTÁS:
        v_x = ADRC_szamitas(hiba_vektor.x, adrc_x)
        v_y = ADRC_szamitas(hiba_vektor.y, adrc_y)
        v_z = 0
        HA |hiba_vektor.x| < XY_KUSZOB ÉS |hiba_vektor.y| < XY_KUSZOB:
            allapot = Z_KÖZELÍTÉS

    HA allapot == Z_KÖZELÍTÉS:
        v_x = ADRC_szamitas(hiba_vektor.x, adrc_x)  # finomkorrekcio
        v_y = ADRC_szamitas(hiba_vektor.y, adrc_y)
        v_z = KOZELITESI_SEBESSEG  # konstans, lassú előre mozgás
        HA hiba_vektor.z < Z_KUSZOB:
            allapot = DOKKOL

    HA allapot == DOKKOL:
        v_x = 0
        v_y = 0
        v_z = DOKKOLAS_SEBESSEG  # nagyon lassú, lineáris mozgás
        HA ero_szenzor > ERO_LIMIT:
            allapot = HIBA_KEZELÉS

    sebesség_parancs = Twist(v_x, v_y, v_z)
    robot_publisher.publish(sebesség_parancs)
    RETURN allapot
```

### 3.2.4 robot_node

**Feladata:** A magas szintű mozgásparancsokat (Cartesian sebesség) ízületi szintű parancsokra konvertálni és továbbítani a robot drivernek.

**Bemenetek:**

| Topic | Típus |
|---|---|
| `/robot/cartesian_velocity` | `geometry_msgs/Twist` |

**Kimenetek:**

| Topic | Típus | Leírás |
|---|---|---|
| `/joint_group_velocity_controller/commands` | `std_msgs/Float64MultiArray` | Ízületi sebességparancsok |
| `/robot/joint_states` | `sensor_msgs/JointState` | Aktuális ízületi állapot |

**Inverz kinematika:**
A Cartesian sebesség → ízületi sebesség konverzióhoz az UR robot Jacobian mátrixát használjuk. Ez a node-ban van implementálva, vagy a MoveIt2 keretrendszer végzi el.

**MoveIt2 integráció:**
A MoveIt2 (`https://moveit.picknik.ai/`) egy ROS2 mozgástervező keretrendszer, amely tartalmaz:
- Inverz kinematika megoldót (KDL, BioIK)
- Ütközésdetektálást
- Pályatervezőt (OMPL algoritmusok)

## 3.3 ROS2 csomagstruktúra

```
visual_servoing_ws/                    ← ROS2 workspace
├── src/
│   ├── visual_servoing/               ← Fő csomag
│   │   ├── visual_servoing/
│   │   │   ├── __init__.py
│   │   │   ├── camera_node.py
│   │   │   ├── vision_node.py
│   │   │   ├── controller_node.py
│   │   │   ├── robot_node.py
│   │   │   ├── ibvs_controller.py    ← IBVS matematika
│   │   │   ├── adrc_controller.py    ← ADRC implementáció
│   │   │   └── feature_detector.py   ← OpenCV alapú detektálás
│   │   ├── launch/
│   │   │   ├── simulation.launch.py  ← Gazebo indítás
│   │   │   └── real_robot.launch.py  ← Valódi hardware
│   │   ├── config/
│   │   │   ├── params.yaml           ← Rendszer paraméterek
│   │   │   └── camera_calibration.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── robot_description/             ← URDF modellek
│   │   ├── urdf/
│   │   │   ├── ur3e.urdf.xacro
│   │   │   └── gripper.xacro
│   │   └── meshes/
│   │
│   └── simulation_world/              ← Gazebo world
│       ├── worlds/
│       │   └── assembly_world.world
│       └── models/
│           ├── wall/
│           └── connector/
│
└── install/                           ← Build kimenet (auto generált)
```

## 3.4 Launch fájl – Szimuláció

A launch fájl egyszerre indítja el az összes szükséges komponenst:

```
LAUNCH simulation.launch.py:
    # 1. Gazebo szimulátor indítása a world fájllal
    INDÍT gazebo_ros gazebo --world assembly_world.world

    # 2. Robot URDF betöltése és spawn
    INDÍT robot_state_publisher --param robot_description=ur3e.urdf
    INDÍT gazebo_ros spawn_entity --robot_name ur3e

    # 3. ROS2 kontrollerek aktiválása
    INDÍT controller_manager joint_state_broadcaster
    INDÍT controller_manager joint_trajectory_controller

    # 4. Alkalmazás node-ok indítása
    INDÍT visual_servoing camera_node
    INDÍT visual_servoing vision_node
    INDÍT visual_servoing controller_node
    INDÍT visual_servoing robot_node

    # 5. RViz2 megjelenítő
    INDÍT rviz2 --config visual_servoing.rviz
```

---

# 4. Vision Alrendszer

## 4.1 Kamera kalibráció

A kamera kalibráció célja a kamera belső (intrinsic) és külső (extrinsic) paramétereinek meghatározása. Ezek nélkül a képi koordináták nem konvertálhatók térbeli koordinátákra, és a homográfia mátrixok sem számíthatók pontosan.

### 4.1.1 Intrinsic paraméterek

A kamera projektív modellje (pinhole camera model) az alábbi összefüggéssel írható le:

```
    [u]   [fx   0  cx] [X/Z]
    [v] = [ 0  fy  cy] [Y/Z]
    [1]   [ 0   0   1] [ 1 ]
```

ahol:
- `(u, v)` – képi koordináták pixelben
- `(X, Y, Z)` – 3D pont a kamera koordinátarendszerben
- `fx, fy` – fókusztávolság pixelben (x és y irányban)
- `cx, cy` – főpont (optical center) koordinátái pixelben

Ezeket az értékeket tartalmazza a **K (intrinsic / camera matrix)**:

```
    K = [fx   0  cx]
        [ 0  fy  cy]
        [ 0   0   1]
```

### 4.1.2 Torzítási együtthatók

A valódi kamerák optikája nem tökéletesen lineáris – radiális és tangenciális torzítást okoz. Ezt az alábbi együtthatókkal írjuk le:

```
Radiális torzítás: k1, k2, k3
Tangenciális torzítás: p1, p2
```

Az OpenCV `calibrateCamera()` függvénye ezeket automatikusan meghatározza.

### 4.1.3 Kalibrációs folyamat

**Szükséges:** Sakktábla minta (pl. 9×6 belső sarok, 25 mm-es négyzetekkel), ~15-20 kép különböző szögekből és pozíciókból.

**Pszeudokód:**

```
FÜGGVÉNY kamera_kalibralas(kepek_listaja):
    obj_pontok = []     ← 3D pontok (sakktábla sarkok ismert pozíciói)
    kep_pontok = []     ← 2D pontok (megtalált sarkok a képen)

    MINDEN kep IN kepek_listaja:
        szurke = szurke_konverzio(kep)
        talalt, sarkok = sakkTabla_sarkok_keresese(szurke, (9,6))
        HA talalt:
            obj_pontok.hozzaad(idealis_sakkTabla_pontok)
            pontositott = subpixel_pontositas(szurke, sarkok)
            kep_pontok.hozzaad(pontositott)

    hiba, K, torzitas, R_vektorok, t_vektorok =
        calibrateCamera(obj_pontok, kep_pontok, kep_merete)

    RETURN K, torzitas
```

**ROS2 eszköz:** A `camera_calibration` csomag (`ros-humble-camera-calibration`) egy grafikus felületet biztosít ehhez a folyamathoz, ahol élő képen hajtható végre a kalibráció.

Dokumentáció: `https://wiki.ros.org/camera_calibration`

### 4.1.4 Homográfia mátrix meghatározása (DLT módszer)

A **Direct Linear Transformation (DLT)** módszer segítségével meghatározható a homográfia mátrix, amely a kamera síkjából egy tetszőleges virtuális képsíkra képezi le a pontokat.

**Mi a homográfia?**
Két kép homográfiával kapcsolódik egymáshoz, ha mindkét kép ugyanazt a sík felületet fényképezi különböző nézőpontokból. A H (3×3) mátrix az összes pontot az egyik síkból a másikba transzformálja:

```
    p' = H * p

    ahol p = [u, v, 1]^T (képi pont homogén koordinátákban)
         p' = [u', v', 1]^T (vetített pont a virtuális síkon)
```

**DLT számítás lépései:**

```
FÜGGVÉNY DLT_homografia(kep_pontok, vilag_pontok):
    # Legalább 4 pontpár szükséges a H mátrix egyértelmű meghatározásához
    # Mindkét ponthalmazt normalizálni kell numerikus stabilitás miatt

    A_matrix = []
    MINDEN (p_kep, p_vilag) IN pontparok:
        # Minden pontpárból 2 sort ad hozzá az A mátrixhoz
        sor1 = [p_vilag.x, p_vilag.y, 1, 0, 0, 0,
                -p_kep.u * p_vilag.x, -p_kep.u * p_vilag.y, -p_kep.u]
        sor2 = [0, 0, 0, p_vilag.x, p_vilag.y, 1,
                -p_kep.v * p_vilag.x, -p_kep.v * p_vilag.y, -p_kep.v]
        A_matrix.hozzaad(sor1, sor2)

    # SVD (Singular Value Decomposition) alapú megoldás
    U, S, Vt = SVD(A_matrix)
    H = Vt[-1].reshape(3, 3)  ← legkisebb szinguláris értékhez tartozó vektor
    H = H / H[2,2]            ← normalizálás

    RETURN H
```

Az OpenCV `findHomography()` függvénye ezt automatikusan elvégzi, RANSAC robusztus becslővel kombinálva, amely kiszűri a hibás pontpárokat (outlier-ek).

## 4.2 Jellemzőkinyerés – Csatlakozóaljzat detektálása

A fali csatlakozóaljzat vizuális azonosítása az IBVS rendszer egyik kulcseleme. A detektálás megbízhatósága közvetlenül befolyásolja az egész rendszer pontosságát és robusztusságát.

### 4.2.1 Előfeldolgozás

```
FÜGGVÉNY elofeldolgozas(szines_kep):
    # Szürkeárnyalatos konverzió
    szurke = BGR_SZURKE_konverzio(szines_kep)

    # Gaussian zajszűrés – eltávolítja a kamerazajt
    # kernel_merete: 5×5, sigma: 1.5
    simított = gaussian_szures(szurke, kernel=5, sigma=1.5)

    # Adaptív hisztogram-egyenlítés (CLAHE)
    # Javítja a kontrasztot inhomogén megvilágítás esetén
    clahe = CLAHE(clip_limit=2.0, tile_grid=(8,8))
    normalt = clahe.alkalmaz(simított)

    RETURN normalt
```

A CLAHE (Contrast Limited Adaptive Histogram Equalization) különösen fontos, mert az épületek falain a megvilágítás egyenetlen lehet, és az egyszerű küszöbérték-alapú szegmentálás ezért sokszor megbízhatatlan.

OpenCV dokumentáció: `https://docs.opencv.org/4.x/d5/daf/tutorial_py_histogram_equalization.html`

### 4.2.2 Él- és kontúrdetektálás

```
FÜGGVÉNY aljzat_detektalo(normalt_kep):
    # Canny éldetektálás
    # alsó_kuszob: 50, felső_kuszob: 150 (ezeket hangolni kell)
    elek = canny_eldetektalas(normalt_kep, also=50, felso=150)

    # Morfológiai lezárás – kis hézagok kitöltése az élekben
    kernel = 3x3_négyzetes_elem
    zárt_elek = morfologiai_lezaras(elek, kernel)

    # Kontúrok keresése
    konturok = kontur_keresese(zárt_elek, mod=EXTERNAL)

    # Szűrés terület és arány alapján
    jeloltek = []
    MINDEN kontur IN konturok:
        terulet = kontur_terulet(kontur)
        hatarolo_teglalapot = min_teglalapot(kontur)
        arany = hatarolo_teglalapot.szelesseg / hatarolo_teglalapot.magassag

        # Csatlakozóaljzat kb. téglalap alakú, 1:1.5 - 1:2 arányban
        HA MIN_TERULET < terulet < MAX_TERULET:
            HA 0.5 < arany < 2.0:
                jeloltek.hozzaad(kontur)

    RETURN legjobb_jelolt(jeloltek)
```

### 4.2.3 Célpont jellemzőpontok meghatározása

A detektált csatlakozóaljzatból négy sarokpontot nyerünk ki, amelyek az IBVS számára a célpont jellemzőit alkotják:

```
FÜGGVÉNY sarokpontok_kinyerese(aljzat_kontur):
    # Közelítő sokszög keresése
    epsilon = 0.02 * kontur_kerulet(aljzat_kontur)
    poligon = poligon_kozzelites(aljzat_kontur, epsilon)

    HA poligon.pontok_szama == 4:
        # Négyszög – rendezzük sorba az óramutató járásával ellentétesen
        sarkok = sorrend_rendezese(poligon.pontok)
        RETURN sarkok
    EGYÉBKÉNT:
        # Határoló téglalapból számítjuk a sarokpontokat
        teglalapot = min_teglalapot(aljzat_kontur)
        RETURN teglalapot.sarkok
```

### 4.2.4 Dugasz pozíció detektálása

A robotkar végén lévő dugaszt is detektálnunk kell. Ehhez két lehetséges megközelítés:

**A) Szín alapú detektálás:** Ha a dugaszt (vagy egy rajta lévő jelölőt) egyedi színnel látjuk el:

```
FÜGGVÉNY dugasz_detektor_szin(kep, cel_szin_HSV):
    hsv_kep = BGR_HSV_konverzio(kep)
    maszk = szin_szures(hsv_kep, cel_szin_HSV.also, cel_szin_HSV.felso)
    konturok = kontur_keresese(maszk)
    RETURN legnagyobb_kontur(konturok).kozeppont
```

**B) Kör/ellipszis detektálás:** Ha a dugasz véglapja kör alakú:

```
FÜGGVÉNY dugasz_detektor_kor(kep):
    szurke = szurke_konverzio(kep)
    # Hough kör transzformáció
    korok = hough_kor_detektalas(
        szurke,
        min_sugár=10,
        max_sugár=50,
        min_tavolsag=100
    )
    HA korok NEM ÜRES:
        RETURN legjobb_kor(korok).kozeppont
```

## 4.3 Hibavektor számítás a virtuális képsíkon

Miután azonosítottuk mind a célpontot (aljzat), mind az aktuális pozíciót (dugasz), a homográfia mátrix segítségével mindkettőt a virtuális képsíkra vetítjük:

```
FÜGGVÉNY hiba_szamitas(aljzat_pontok, dugasz_pontok, H_a, H_b):
    # Vetítés a virtuális képsíkra
    aljzat_virtualis = homografia_alkalmaz(H_a, aljzat_pontok)
    dugasz_virtualis = homografia_alkalmaz(H_b, dugasz_pontok)

    # Középpontok számítása
    aljzat_kozep = atlag(aljzat_virtualis)
    dugasz_kozep = atlag(dugasz_virtualis)

    # Hibavektor (képtérbeli eltérés pixelben)
    hiba_x = aljzat_kozep.x - dugasz_kozep.x
    hiba_y = aljzat_kozep.y - dugasz_kozep.y

    # Z irányú hiba – az aljzat látszó méretének változásából
    aljzat_terulet_aktualis = kontur_terulet(aljzat_pontok)
    hiba_z = REFERENCIA_TERULET - aljzat_terulet_aktualis
    # Ha az aljzat nagy a képen → közel van → kis hiba
    # Ha az aljzat kicsi → messze van → nagy hiba

    RETURN (hiba_x, hiba_y, hiba_z)
```

---

# 5. IBVS Vezérlő – Matematikai Alapok

## 5.1 A visual servoing alapgondolata

A visual servoing (vizuális szervovezérlés) lényege, hogy a robot mozgását nem előre tervezett koordináták, hanem a kamera által látott képi jellemzők alapján vezéreljük. A cél: a jelenlegi képi jellemzők értékét `s` közelítsük a kívánt értékhez `s*`.

Az **Image-Based Visual Servoing (IBVS)** esetén minden számítás a képtérben (2D) történik – nem szükséges a 3D-s rekonstrukció.

## 5.2 Képi jellemzők és a hibavektor

Legyen `s` az aktuális képi jellemzők vektora, `s*` a kívánt értékek vektora. A hibavektor:

```
e(t) = s(t) - s*
```

A célunk: `e → 0`, azaz a hibavektor nullává válik, mikor a dugasz pontosan az aljzat előtt van.

A képi jellemzők lehetnek:
- Pontok koordinátái: `s = (u₁, v₁, u₂, v₂, ...)`
- Területek, momentumok, vonalak paraméterei

Rendszerünkben az aljzat és a dugasz középpontjainak különbségét, valamint a méretarány eltérést (Z irányhoz) használjuk jellemzőként.

## 5.3 Az interakciós mátrix (Image Jacobian)

Az IBVS vezérlő kulcseleme az **interakciós mátrix** (más névvel: feature Jacobian vagy image Jacobian), jelölése `L`. Ez a mátrix leírja, hogyan változnak a képi jellemzők a kamera (illetve a robot end-effector) mozgásának hatására.

Egy képi pont `(u, v)` esetén az interakciós mátrix:

```
L_pont = [-fx/Z    0      u/Z    u*v/fx    -(fx² + u²)/fx    v ]
         [  0    -fy/Z    v/Z  (fy² + v²)/fy    -u*v/fy     -u ]
```

ahol:
- `fx, fy` – fókusztávolság (a K mátrixból)
- `u, v` – a képi pont koordinátái (normalizált, azaz főponthoz képest mérve)
- `Z` – a 3D pont mélysége (kamera koordinátarendszerben)

Ez a mátrix 2×6 méretű, mert:
- 2 sor: az (u, v) koordináták változása
- 6 oszlop: a 6 lehetséges mozgás komponens: `(vx, vy, vz, ωx, ωy, ωz)`

Ha több jellemzőpontot használunk, az interakciós mátrixok egymás alá rakva alkotják a teljes L mátrixot.

## 5.4 A vezérlési törvény

Az IBVS vezérlési törvénye exponenciális konvergenciát biztosít:

```
v_c = -λ * L⁺ * e
```

ahol:
- `v_c` – a kamera (end-effector) sebességparancs: `[vx, vy, vz, ωx, ωy, ωz]^T`
- `λ` – vezérlési erősítés (gain), tipikusan 0.1 – 1.0
- `L⁺` – az interakciós mátrix pszeudo-inverze: `L⁺ = (L^T * L)⁻¹ * L^T`
- `e` – hibavektor

Ez azt jelenti, hogy ha a hibavektor nagy, a robot gyorsan mozdul; ha közel van a célhoz, lassít. Az exponenciális konvergencia garantálja, hogy `e → 0` ahogy `t → ∞`.

**A pszeudo-inverz szükségessége:**
Az L mátrix általában nem négyzetes (több jellemző esetén több sora van, mint oszlopa), ezért nem invertálható közvetlenül – pszeudo-inverzt kell számítani. Numerikusan stabilis módszer az SVD alapú pszeudo-inverzálás.

## 5.5 Perspektív transzformáció alapú IBVS (Cao 2022)

A hagyományos IBVS egyik problémája, hogy ha a robot mozgása nagy, a kamera látómezőből kieshetnek a jellemzők, vagy a robot fizikai korlátokba ütközhet. A Cao (2022) által javasolt megközelítés ezt a problémát kezeli.

### A virtuális képsík koncepciója

```
                    Virtuális képsík (γ̃)
                    ┌─────────────────┐
                    │                 │
    [KAMERA] ──────►│  f̃_α  f̃_β      │
                    │     ↕           │
                    │  (hiba=0 →      │
                    │   illesztve)    │
                    └─────────────────┘
                    ↑
              H_a és H_b mátrixok vetítik ide
              az aljzat és dugasz pontjait
```

**Lépések:**

**1. Virtuális képsík létrehozása:**
A kalibrációs fázisban meghatározunk egy virtuális síkot `γ̃`, amely párhuzamos a célfelülettel (a fallal). Két homográfia mátrixot számítunk:
- `H_a`: az aljzat képi pontjait vetíti a virtuális síkra → `f̃_α`
- `H_b`: a dugasz képi pontjait vetíti a virtuális síkra → `f̃_β`

**2. Illesztési feltétel:**
Az összeszerelésnél az feltétel teljesül, ha `f̃_α = f̃_β`, azaz mindkét objektum ugyanazon a ponton van a virtuális síkon. Ez azt jelenti, hogy a dugasz pontosan az aljzat előtt van, és csak Z irányban különböznek (mélységi eltérés).

**3. Hibavektor a virtuális síkon:**
```
e_virtualis = f̃_α - f̃_β

e_virtualis.x = aljzat_virtualis.x - dugasz_virtualis.x
e_virtualis.y = aljzat_virtualis.y - dugasz_virtualis.y
```

**4. Előny a hagyományos IBVS-sel szemben:**
A virtuális képsíkon számított hibavektor monoton csökken a mozgás során, és nem okoz szinguláris állapotokat. Ez robusztusabb konvergenciát eredményez, különösen nagy kezdeti eltérések esetén.

## 5.6 Mélységi (Z irányú) vezérlés

A Z irányú vezérlés a perspektív transzformáció keretrendszerén belül az aljzat látszó méretének változásán alapul:

```
Mélység becslés jellemzőméret alapján:

    Z_becsült = (f * D_valódi) / D_képen

ahol:
    f        = fókusztávolság (ismert a kalibrációból)
    D_valódi = az aljzat valódi mérete (ismert, pl. 80mm)
    D_képen  = az aljzat látszó mérete pixelben (mérhető)

Hiba:
    e_z = Z_becsült - Z_kívánt

ahol Z_kívánt a dokkolási mélység (kalibráláskor meghatározva)
```

Alternatívaként: ha Intel RealSense kamerát használunk, a mélységi csatorna közvetlenül megadja Z értékét mérési zajtól eltekintve.

## 5.7 Dokkolási fázis – Attitűd-kivonásos módszer

Miután az XY igazítás és a Z közelítés megtörtént, a végső dokkolási lépés lineáris, egyenes pályán történik. Az attitűd-kivonásos módszer lényege:

**1.** Az end-effector aktuális orientációját (forgatás mátrix R) kiszámítjuk a robot kinematikájából.

**2.** A dugasz orientációját igazítjuk az aljzat síkjának normálisához (azaz a dugasz egyenesen néz a lyukba).

**3.** Lineáris mozgás Z irányban, az orientáció megtartásával.

```
FÜGGVÉNY dokkolasi_mozgas(cel_melyseg, aktualis_orientacio):
    AMÍG aktualis_Z < cel_melyseg:
        # Orientáció korrekció (ha szükséges)
        orientacio_hiba = cel_orientacio - aktualis_orientacio
        HA |orientacio_hiba| > ORIENTACIO_KUSZOB:
            orientacio_korrekcios_mozgas(orientacio_hiba)

        # Lineáris előre mozgás
        v_z = DOKKOLAS_SEBESSEG  # pl. 5 mm/s
        mozgas_parancs_kuldese(0, 0, v_z, 0, 0, 0)

        # Erőellenőrzés (biztonság)
        HA ero_szenzor() > MAX_ERO:
            LEALLITAS()
            HIBA_JELZESE("Mechanikai akadály")
            RETURN SIKERTELEN

    RETURN SIKERES
```

---

# 6. ADRC Vezérlő

## 6.1 Miért ADRC?

A hagyományos PID vezérlő az IBVS rendszerekben megfelelően működik egyszerű körülmények között, de problémákat okoznak:
- **Modellezési hibák:** A robot dinamikája nem pontosan ismert
- **Külső zavarok:** Rezgések, nem egyenletes terhelés
- **Nem-linearitások:** A robot Jacobian mátrix pozíciófüggő

Az **Active Disturbance Rejection Control (ADRC)** ezeket a problémákat kezeli azáltal, hogy aktívan becsüli és kompenzálja a zavarokat, anélkül hogy pontos rendszermodellre lenne szüksége.

Az ADRC-t Han Jingqing fejlesztette ki az 1990-es években, és az IBVS rendszerekben Cao (2022) alkalmazta sikeresen.

## 6.2 Az ESO – Extended State Observer

Az ADRC lelke a **Kiterjesztett Állapotmegfigyelő (ESO)**, amely a rendszer állapotai mellett a zavarokat is becsüli. Tekintsük a következő egyszerűsített rendszermodellt:

```
ÿ = f(y, ẏ, d, t) + b * u
```

ahol:
- `y` – kimenet (pl. képi koordináta)
- `u` – bemeneti vezérlőjel (pl. sebesség parancs)
- `f(...)` – az összes ismert és ismeretlen dinamika + zavar (ezt becsüli az ESO)
- `b` – bemeneti erősítés (közelítőleg ismert)
- `d` – külső zavar

Az ESO állapotegyenletei:

```
Állapotok:
    z1 ≈ y         (kimenet becslés)
    z2 ≈ ẏ         (kimenet derivált becslése)
    z3 ≈ f(...)    (teljes zavar becslése)

ESO dinamika:
    ż1 = z2 - β1 * (z1 - y)
    ż2 = z3 - β2 * fal(z1 - y, α1, δ)  + b * u
    ż3 =    - β3 * fal(z1 - y, α2, δ)

ahol:
    β1, β2, β3 – observer erősítések (hangolási paraméterek)
    fal(...)   – nemlineáris függvény (a robusztusságot növeli)
    α1, α2, δ  – a nemlineáris függvény paraméterei
```

A lineáris ESO (LESO) esetén a `fal()` helyett egyszerű szorzót használunk, ami könnyebben hangolható, de kissé kevésbé robusztus.

## 6.3 Nemlineáris Állapot-Visszacsatolás (NLSEF)

Az ESO által becsült állapotok alapján a vezérlőjel:

```
# Hibák számítása
e1 = r - z1      ← referencia - becsült kimenet
e2 = ṙ - z2      ← referencia derivált - becsült derivált (ṙ=0 állandó célnál)

# Nemlineáris kombinálás
u0 = β01 * fal(e1, α01, δ0) + β02 * fal(e2, α02, δ0)

# Zavar kompenzáció
u = (u0 - z3) / b
```

Ez a vezérlőjel kompenzálja a z3-ban becsült zavarokat, így a zárt hurkú rendszer zavarrobusztus lesz.

## 6.4 ADRC hangolása

Az ADRC néhány paramétert igényel, amelyeket hangolni kell:

```yaml
# config/params.yaml – ADRC paraméterek
adrc:
  # ESO sávszélesség (ω_o) – ez határozza meg az observer sebességét
  # Általános szabály: ω_o = 5-10 * zárt_hurok_sávszélesség
  observer_bandwidth: 50.0  # rad/s

  # Zárt hurok sávszélesség (ω_c)
  # Meghatározza a vezérlő sebességét
  control_bandwidth: 10.0  # rad/s

  # Bemeneti erősítés becslés
  b0: 1.0  # hangolni kell a konkrét rendszerre

  # Nemlinearitás paraméterei
  alpha1: 0.5
  alpha2: 0.25
  delta: 0.01
```

A hangolási folyamat során először a szimulációban keressük meg az optimális értékeket, majd valódi hardveren finomítjuk.

## 6.5 Összefoglalt vezérlő struktúra

```
Referencia (s*)
     │
     │ e = s* - s
     ▼
┌──────────┐    u0    ┌──────────┐
│  NLSEF   │─────────►│  Zavar   │   u    ┌─────────┐
│ (hiba →  │          │kompenz.  │────────►│  ROBOT  │──► y (mozgás)
│  vezérlő)│          │u=(u0-z3)/b│        └────┬────┘
└──────────┘          └──────────┘              │
     ▲                      ▲                   │
     │ z1, z2               │ z3                │ y (mért kimenet)
     └──────────────────────┴───────────────────┘
                       ┌──────────┐
                       │   ESO    │
                       │(extended │
                       │  state   │
                       │observer) │
                       └──────────┘
```

---

# 7. Szimulációs Környezet

## 7.1 Gazebo Fortress áttekintés

A **Gazebo** (jelenleg Gazebo Fortress a ROS2 Humble-lel kompatibilis verzió) egy nyílt forráskódú, fizikát szimuláló 3D-s robotikai szimulátorkörnyezet. Főbb jellemzői:

- **Fizikai motor:** ODE (Open Dynamics Engine) vagy Bullet – szimulálj ütközéseket, súrlódást, gravitációt
- **Szenzor szimuláció:** Kamera, LIDAR, IMU, erőszenzor – mind szimulálhatók
- **ROS2 integráció:** A `ros_gz_bridge` csomag topic-okon keresztül kapcsolja össze Gazebo-t és ROS2-t
- **SDF formátum:** A modellek és world-ök Simulation Description Format (SDF) XML-ben vannak leírva

**Dokumentáció:** `https://gazebosim.org/docs/fortress/`

## 7.2 World fájl struktúra

A szimulációs környezet egy egyszerű irodai/épületi helyszínt reprezentál:

```xml
<!-- worlds/assembly_world.world – SDF formátum (pszeudo) -->
WORLD assembly_world:
    # Fizika beállítások
    FIZIKA:
        gravitáció: [0, 0, -9.81]
        lépésköz: 0.001 s (1 kHz)

    # Megvilágítás
    FÉNY típus=napfény:
        irány: [0.5, 0.1, -0.9]

    # Talaj
    MODELL talaj:
        GEOMETRIA sík

    # Fal
    MODELL fal:
        POZÍCIÓ: [1.0, 0.0, 0.75]  ← 1m-re a robottól
        MÉRET: [0.1, 2.0, 1.5]     ← vastag:2m széles:1.5m magas
        ANYAG: fehér_festett_fal

    # Fali csatlakozóaljzat (célpont)
    MODELL aljzat:
        SZÜLŐ: fal
        POZÍCIÓ: [0.0, 0.0, 0.3]   ← falra relatív, padlótól 30cm
        STL_MESH: "models/connector_socket.stl"
        ANYAG: fehér_műanyag

    # Robot (spawn-olódik indításkor)
    INCLUDE:
        URI: "model://ur3e"
        POZÍCIÓ: [0.0, -0.3, 0.0]  ← faltól 1.3m-re

    # Szimulált kamera (fix pozícióban)
    MODELL kamera_tartó:
        POZÍCIÓ: [0.5, -0.5, 0.8]  ← oldalt, magasabban
        SENSOR kamera:
            TÍPUS: kamera
            FELBONTÁS: [1280, 720]
            FPS: 30
            LÁTÓSZÖG: 69 fok
            ROS2_TOPIC: "/camera/image_raw"
```

## 7.3 Robot modell (URDF / xacro)

A robot leírása URDF (Unified Robot Description Format) formátumban történik, xacro makrók segítségével (amelyek újrafelhasználható komponenseket tesznek lehetővé).

```
ROBOT ur3e_visual_servoing:

    # Bázis link (a robot rögzített alapja)
    LINK bázis:
        INERCIA: [...]
        VIZUÁLIS: ur3e_base.stl
        ÜTKÖZÉSI: egyszerűsített_henger

    # 6 ízület és 6 link (a UR3e kinematikája alapján)
    ÍZÜLET shoulder_pan:  TÍPUS: folyamatos_forgás, TENGELY: Z
    LINK  upper_arm
    ÍZÜLET shoulder_lift: TÍPUS: folyamatos_forgás, TENGELY: Y
    LINK  forearm
    ÍZÜLET elbow:         TÍPUS: folyamatos_forgás, TENGELY: Y
    ...

    # End-effector (csatlakozódugasz tartó)
    LINK end_effector:
        VIZUÁLIS: dugasz_tartó.stl
        ÜTKÖZÉSI: henger(r=0.02, h=0.05)

    # Gazebo bővítmények
    GAZEBO_PLUGIN: ros2_control (ízületi vezérlőknek)
    GAZEBO_PLUGIN: joint_state_publisher (állapot olvasáshoz)
```

**Meglévő UR3e URDF:** A Universal Robots hivatalos `ur_description` ROS2 csomag tartalmazza a teljes URDF modellt, amelyet közvetlenül felhasználhatunk:
`https://github.com/UniversalRobots/Universal_Robots_ROS2_Description`

## 7.4 ROS2 Kontroller konfiguráció

A `ros2_control` keretrendszer kezeli a szimulált (és valódi) robot ízületi vezérlőit:

```yaml
# config/ros2_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 125  # Hz

joint_state_broadcaster:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint

joint_trajectory_controller:
  ros__parameters:
    joints: [shoulder_pan_joint, ...]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    allow_partial_joints_goal: false
```

## 7.5 RViz2 vizualizáció

Az RViz2 valós idejű megjelenítést biztosít:
- **TF fa:** Koordinátarendszerek hierarchiája (robot, kamera, végpont)
- **Kamera kép:** A `vision_node` által annotált kép (detektált jellemzők, hibavektor)
- **Robot modell:** 3D megjelenítés az aktuális pozícióban
- **Pályatervek:** A tervezett mozgások megjelenítése

```
# RViz2 konfiguráció elemei:
- Kijelző: RobotModel (URDF alapján)
- Kijelző: Image (/vision/debug_image topic)
- Kijelző: TF (koordinátarendszerek)
- Kijelző: Path (/robot/planned_path)
- Kijelző: Marker (célpont jelölése a 3D-s térben)
```

---

# 8. Tesztelési Terv

## 8.1 Tesztelési szintek

### 8.1.1 Egységtesztek (Unit Tests)

Minden szoftver modul önállóan tesztelendő:

| Modul | Teszt | Elvárt eredmény |
|---|---|---|
| `feature_detector.py` | Szintetikus kép bementre detektál-e aljzatot | 100% detektálás kontrollált képeken |
| `ibvs_controller.py` | Ismert hibavektorra helyes vezérlőjel | Mathematikailag ellenőrizhető |
| `adrc_controller.py` | Lépés bemenetre konvergál-e | Oszcilláció nélküli konvergencia |
| `homography.py` | Ismert pontokból helyes H mátrix | Visszavetítési hiba < 1 pixel |

### 8.1.2 Integrációs tesztek

| Teszt | Leírás | Sikerkritérium |
|---|---|---|
| Vision pipeline end-to-end | Képtől hibavektórig teljes pipeline | Hiba < 5 pixel ismert pozíciónál |
| Vezérlő + robot szimuláció | Gazebo-ban a robot eléri a célpozíciót | Pozíciós hiba < 5 mm |
| Teljes rendszer (szimulált) | Teljes illesztési feladat szimulációban | Sikeres illesztés > 90% |

### 8.1.3 Rendszertesztek (valódi hardware, laborban)

| Teszt | Feltételek | Sikerkritérium |
|---|---|---|
| Statikus célpont illesztés | Fix aljzat, ismert megvilágítás | Pozíciós hiba < 2 mm |
| Változó megvilágítás | Különböző fényerő, szögek | Detektálás 80%+ |
| Különböző kezdőpozíciók | ±50 mm eltérés az ideális pozíciótól | Sikeres illesztés > 85% |
| Ismétlési pontosság | 10 egymást követő illesztés | Szórás < 1 mm |

## 8.2 Mérési metrikák

```
1. Pozíciós hiba: d = √(Δx² + Δy² + Δz²)  [mm]
   - Cél: d < 2 mm

2. Sikerességi arány: S = sikeres_illesztések / összes_kísérlet * 100%
   - Cél: S > 90%

3. Konvergencia idő: T = t_illesztés - t_start  [s]
   - Cél: T < 10 s

4. Képfeldolgozás sebessége: fps_vision  [Hz]
   - Cél: fps_vision ≥ 10 Hz (real-time működéshez)

5. Detektálási megbízhatóság: D = sikeres_detektálások / képkockák * 100%
   - Cél: D > 95% kontrollált körülmények között
```

## 8.3 Szimulációs tesztek végrehajtása

```
TESZT eljaras:

1. Gazebo indítása:
   ros2 launch visual_servoing simulation.launch.py

2. Kiindulóhelyzet beállítása:
   ros2 service call /robot/move_to_home

3. Vizualizáció ellenőrzése RViz2-ben:
   - Látható-e a kamera kép?
   - Detektált-e a célpont?

4. Illesztési feladat indítása:
   ros2 action send_goal /assembly/start AssemblyAction

5. Eredmény mérése:
   ros2 topic echo /assembly/result
   → final_error: {x: ..., y: ..., z: ...}
   → success: true/false
   → duration: ...
```

---

# 9. Kockázatelemzés

## 9.1 Technikai kockázatok

| Kockázat | Valószínűség | Hatás | Kezelés |
|---|---|---|---|
| Célpont detektálás meghibásodik változó fénynél | Közepes | Magas | CLAHE + több jellemző kombinálása |
| Homográfia mátrix pontatlan | Alacsony | Magas | DLT + RANSAC + újrakalibráció |
| Robot ütközik a fallal | Alacsony | Kritikus | Erőszenzor figyelés + biztonsági leállás |
| Konvergencia hiánya (IBVS) | Közepes | Közepes | ADRC + vezérlési erősítés hangolása |
| Valódi hardware eltérés szimulációtól | Magas | Közepes | Szimulációs réstől (sim-to-real gap) való tudatos tervezés |

## 9.2 Biztonsági mechanizmusok

```
BIZTONSÁGI RENDSZER:

1. Erőhatárkorlát:
   HA ero_szenzor > 20N:
       AZONNALI_LEALLITAS()

2. Munkaterület korlát:
   HA pozicio KÍVÜL(biztonságos_zóna):
       MOZGAS_LEALLITAS()

3. Detektálási időtúllépés:
   HA célpont_nem_látható > 2 másodperce:
       VISSZATER_VARAKOZASI_POZICIOBA()

4. Sebességkorlát:
   max_lineáris_sebesség = 0.1 m/s
   max_forgási_sebesség = 0.5 rad/s
```

---

# 10. Irodalom és Hivatkozások

## Tudományos cikkek

**[1]** Cao C. *Research on a Visual Servoing Control Method Based on Perspective Transformation under Spatial Constraint.* Machines. 2022; 10(11):1090. DOI: 10.3390/machines10111090

**[2]** Pomares J. *Visual Servoing in Robotics.* Electronics. 2019; 8(11):1298. DOI: 10.3390/electronics8111298

**[3]** Yan S, Tao X, Xu D. *High-precision robotic assembly system using three-dimensional vision.* International Journal of Advanced Robotic Systems. 2021;18(3). DOI: 10.1177/17298814211027029

## Szoftver dokumentáció

**[4]** ROS2 Humble dokumentáció: `https://docs.ros.org/en/humble/`

**[5]** OpenCV 4.x dokumentáció: `https://docs.opencv.org/4.x/`

**[6]** Gazebo Fortress dokumentáció: `https://gazebosim.org/docs/fortress/`

**[7]** Universal Robots ROS2 Driver: `https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver`

**[8]** MoveIt2 dokumentáció: `https://moveit.picknik.ai/`

**[9]** Intel RealSense ROS2 wrapper: `https://github.com/IntelRealSense/realsense-ros`

**[10]** ros2_control dokumentáció: `https://control.ros.org/humble/`

**[11]** camera_calibration ROS2 csomag: `https://github.com/ros-perception/image_pipeline`

## Könyvek

**[12]** Corke P. *Robotics, Vision and Control.* Springer, 2017. – Részletes bevezetés a visual servoing matematikájába

**[13]** Szeliski R. *Computer Vision: Algorithms and Applications.* Springer, 2022. – Homográfia, kamerakalibrálás részletes tárgyalása (ingyenesen elérhető: `https://szeliski.org/Book/`)
