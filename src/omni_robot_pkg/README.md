# omni_robot_pkg

3-tekerli omni yönlü robot için ROS 2 Jazzy kontrol paketi.

**Görev:** Robot çalışınca LiDAR ile en uzak noktayı bulur, oraya gider (50 cm önünde durur), başlangıç noktasına geri döner ve motorları kapatır. Yolda engel çıkarsa DWA algoritmasıyla kaçınır.

---

## Donanım

| Parça | Detay |
|---|---|
| Tekerler | 3× omni, L = 25 cm, r = 5 cm, 750 tick/tur |
| Motor sürücü | 2× RoboClaw 2×15 A — aynı UART hattında multi-drop (`/dev/ttyAMA0`) |
| LiDAR | YDLidar X2 (`/dev/ttyUSB0`) |
| Kamera | Intel RealSense D435b (bağlı ama aktif görevde kullanılmıyor) |
| Bilgisayar | Raspberry Pi 4 |

RoboClaw bağlantısı: GPIO 14 (TX) → pin 8, GPIO 15 (RX) → pin 10, GND → pin 6.
Her iki RoboClaw aynı hatta, adres 0x80 ve 0x81 ile ayrışır.

---

## Raspberry Pi UART Kurulumu (bir kez yapılır)

```bash
sudo nano /boot/firmware/config.txt
```

Şunları ekle:

```
dtoverlay=disable-bt
enable_uart=1
```

Kaydet, yeniden başlat:

```bash
sudo reboot
```

---

## Kurulum (ilk kez)

```bash
# Bağımlılıklar
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-xacro \
     ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
     python3-numpy python3-serial

# Repoyu çek
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/emre-tasocak/ros_yeni_sim_gorev.git

# Paketi derle
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omni_robot_pkg --symlink-install
source install/setup.bash
```

---

## Güncelleme (sonraki seferler)

```bash
cd ~/ros2_ws/src/ros_yeni_sim_gorev && git pull
cd ~/ros2_ws
colcon build --packages-select omni_robot_pkg --symlink-install
source install/setup.bash
```

---

## Gerçek Robot — Çalıştırma

**Terminal 1:**

```bash
sudo chmod 666 /dev/ttyAMA0 /dev/ttyUSB0
source ~/ros2_ws/install/setup.bash
ros2 launch omni_robot_pkg robot_bringup.launch.py
```

**Terminal 2** (4 saniye sonra):

```bash
source ~/ros2_ws/install/setup.bash
ros2 service call /start_mission std_srvs/srv/Trigger
```

RViz ile izlemek için:

```bash
ros2 launch omni_robot_pkg robot_bringup.launch.py rviz:=true
```

---

## Simülasyon (Gazebo Harmonic)

**Terminal 1:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch omni_robot_pkg simulation.launch.py
```

**Terminal 2** (9 saniye sonra):

```bash
source ~/ros2_ws/install/setup.bash
ros2 service call /start_mission std_srvs/srv/Trigger
```

GUI ve RViz olmadan:

```bash
ros2 launch omni_robot_pkg simulation.launch.py gui:=false rviz:=false
```

---

## Görev Durumları

```
IDLE → SCANNING → NAVIGATING_TO_TARGET → AT_TARGET → RETURNING_HOME → DONE
```

Görev durumunu izle:

```bash
ros2 topic echo /mission_state
```

Acil durdur:

```bash
ros2 service call /stop_mission std_srvs/srv/Trigger
```

---

## Düğüm Mimarisi

```
[lidar_node / Gazebo gpu_lidar]
        │
        ▼ /scan
[lidar_processor]──► /farthest_point
        │            /nearest_obstacle
        ▼ /scan/filtered
[obstacle_avoidance]◄── /cmd_vel_nav ──[navigation_node]◄── /goal_pose ──[mission_node]
        │                                      │                               │
        ▼ /cmd_vel                        /goal_reached                   /odom (pozisyon)
[roboclaw_driver / Gazebo VelocityControl]
        │
        ▼ /wheel_ticks  (sadece gerçek robot)
[odometry_node] ──► /odom + TF(odom→base_link)

Simülasyonda:
[Gazebo OdometryPublisher] → ros_gz_bridge → /odom → [sim_odom_tf_node] → TF
```

---

## Parametreler

Tüm parametreler `config/robot_params.yaml` dosyasındadır:

| Parametre | Değer | Açıklama |
|---|---|---|
| `obstacle_safety_radius` | 0.10 m | Robota minimum yaklaşma mesafesi |
| `mission_stop_distance` | 0.50 m | Hedefe ne kadar uzakta durulsun |
| `lidar_min_range` | 0.30 m | Bu mesafenin altı filtrelenir (tekerler) |
| `max_linear_velocity` | 0.30 m/s | Maksimum hız |
| `dwa_min_clearance` | 0.08 m | DWA engel toleransı |
| `roboclaw_port_1` | `/dev/ttyAMA0` | RoboClaw UART portu (multi-drop) |
| `roboclaw_baudrate` | 38400 | RoboClaw baud hızı |

---

## Hata Ayıklama

```bash
# cmd_vel geliyor mu?
ros2 topic echo /cmd_vel

# LiDAR çalışıyor mu?
ros2 topic echo /scan

# Enkoder verisi var mı?
ros2 topic echo /wheel_ticks

# Odometri
ros2 topic echo /odom
```
