# 00 – Telepítés: ROS2 Jazzy + Gazebo Harmonic

**Rendszer:** Ubuntu 24.04 LTS (WSL2)
**ROS2 verzió:** Jazzy Jalisco (az Ubuntu 24.04-hez való LTS)
**Gazebo verzió:** Harmonic (Jazzy-vel kompatibilis)

---

## 1. lépés – Locale beállítása

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Ellenőrzés:
```bash
locale
# Látnia kell: LANG=en_US.UTF-8
```

---

## 2. lépés – ROS2 repository hozzáadása

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

---

## 3. lépés – ROS2 Jazzy telepítése

```bash
# Teljes desktop verzió (RViz2-vel, demókkal)
sudo apt install -y ros-jazzy-desktop

# Fejlesztői eszközök
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
```

Ez kb. 10-15 percet vesz igénybe, ~3 GB letöltés.

---

## 4. lépés – ROS2 automatikus betöltése

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Ellenőrzés:
```bash
ros2 --version
# Várható kimenet: ros2 jazzy (valami dátum)
```

---

## 5. lépés – Gazebo Harmonic telepítése

```bash
# Gazebo repo kulcs
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic
```

Ellenőrzés:
```bash
gz sim --version
# Várható: Gazebo Harmonic ...
```

---

## 6. lépés – ROS2–Gazebo bridge telepítése

Ez az összekötő csomag, ami lehetővé teszi, hogy a ROS2 és Gazebo kommunikáljanak egymással.

```bash
sudo apt install -y ros-jazzy-ros-gz
```

---

## 7. lépés – rosdep inicializálása

```bash
sudo rosdep init
rosdep update
```

---

## 8. lépés – Tesztelés

### ROS2 teszt:
```bash
# Egy terminálban:
ros2 run demo_nodes_cpp talker

# Másik terminálban:
ros2 run demo_nodes_py listener
```

Látnia kell: `[talker]: Publishing: 'Hello World: 1'` és `[listener]: I heard: [Hello World: 1]`

### Gazebo teszt:
```bash
gz sim
```

Megnyílik a Gazebo ablak (üres világ). Ha látod, minden rendben.

---

## Hibakezelés

### Ha a Gazebo nem nyílik meg (display hiba):
```bash
# Ellenőrzés:
echo $DISPLAY
# Kell látni: :0

# Ha üres:
export DISPLAY=:0
```

### Ha lassú az indítás WSL2-ben:
Ez normális jelenség – az első indítás 30-60 másodperc is lehet.

### Ha `locale` hibaüzenetet kapsz ROS2-nél:
```bash
export LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8
```

---

## Következő lépés

Ha minden sikerült → [01_ros2_alapok.md](01_ros2_alapok.md)
