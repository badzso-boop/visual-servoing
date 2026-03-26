#!/bin/bash
# =============================================================
# ROS2 Jazzy + Gazebo Harmonic telepítő script
# Ubuntu 24.04 WSL2-re
# =============================================================
set -e  # leáll hiba esetén

echo "================================================"
echo "  ROS2 Jazzy + Gazebo Harmonic telepítő"
echo "================================================"
echo ""

# --- 1. Locale ---
echo "[1/7] Locale beállítása..."
sudo apt update -qq
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# --- 2. ROS2 repo ---
echo "[2/7] ROS2 repository hozzáadása..."
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update -qq

# --- 3. ROS2 Jazzy telepítése ---
echo "[3/7] ROS2 Jazzy Desktop telepítése... (ez 10-15 percig tarthat)"
sudo apt install -y ros-jazzy-desktop

# --- 4. Fejlesztői eszközök ---
echo "[4/7] Fejlesztői eszközök..."
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro

# --- 5. Gazebo Harmonic ---
echo "[5/7] Gazebo Harmonic telepítése..."
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update -qq
sudo apt install -y gz-harmonic

# --- 6. ROS2-Gazebo bridge ---
echo "[6/7] ROS2-Gazebo bridge telepítése..."
sudo apt install -y ros-jazzy-ros-gz

# --- 7. rosdep ---
echo "[7/7] rosdep inicializálása..."
sudo rosdep init 2>/dev/null || echo "(rosdep már inicializálva)"
rosdep update

# --- .bashrc frissítése ---
echo ""
echo "Bashrc frissítése..."
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  echo "  + ROS2 source hozzáadva a .bashrc-hez"
fi

# --- Ellenőrzés ---
echo ""
echo "================================================"
echo "  Telepítés kész! Ellenőrzés:"
echo "================================================"
source /opt/ros/jazzy/setup.bash

echo -n "ROS2: "
ros2 --version 2>/dev/null | head -1 || echo "HIBA"

echo -n "Gazebo: "
gz sim --version 2>/dev/null | head -1 || echo "HIBA"

echo ""
echo "Ha mindkettő OK, futtasd:"
echo "  source ~/.bashrc"
echo "  ros2 run demo_nodes_cpp talker"
echo ""
echo "Majd egy másik terminálban:"
echo "  ros2 run demo_nodes_py listener"
