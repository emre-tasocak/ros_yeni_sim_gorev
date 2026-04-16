# Omni Robot – ROS2 Jazzy

3-tekerlekli omnidirectional robot: Gazebo Harmonic simülasyonu ve gerçek donanım (RoboClaw 2×15A).

## Paketler

| Paket | Açıklama |
|---|---|
| `omni_robot` | Kinematik kontrolcü (`/cmd_vel` → tekerlek hızları) |
| `omni_robot_sim` | Gazebo simülasyonu, sim_kinematics, roboclaw_hw sürücüsü, RRT, path planner |
| `omni_mission` | LiDAR tabanlı görev düğümleri (sim + hw) |

## Kurulum

### Gereksinimler

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  python3-numpy \
  python3-colcon-common-extensions
```

### Klonlama ve Derleme

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/KULLANICI_ADINIZ/omni_robot_ws.git src
colcon build --symlink-install
source install/setup.bash
```

## Çalıştırma

### Simülasyon (Gazebo)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch omni_mission mission_sim.launch.py
```

### Gerçek Robot

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch omni_mission mission_hardware.launch.py \
  roboclaw_port:=/dev/ttyUSB0 \
  lidar_port:=/dev/ttyUSB1
```

## Robot Kinematik Parametreleri

- Tekerlek yarıçapı: **r = 0.05 m**
- Robot yarıçapı (L): **L = 0.20 m**
- Tekerlek açıları: **α₁ = -60°, α₂ = +60°, α₃ = 180°**
- RoboClaw encoder: **750 tick/tur** (25 CPR × 30:1)
- Baud: **38400**, Adres: 0x80 (motor 1+2), 0x81 (motor 3)
