# omni_robot_pkg

3-tekerli omni yönlü robot için ROS 2 Jazzy kontrol paketi.

**Görev:** Robot çalışınca LiDAR ile en uzak noktayı bulur, oraya gider (50 cm önünde durur), başlangıç noktasına geri döner ve motorları kapatır. Yolda engel çıkarsa DWA algoritmasıyla kaçınır.

---

## Donanım

| Parça | Detay |
|---|---|
| Tekerler | 3× omni, L = 25 cm, r = 5 cm, 750 tick/tur |
| Motor sürücü | 2× RoboClaw 2×15 A (`/dev/ttyAMA0`, `/dev/ttyAMA1`) |
| LiDAR | YDLidar X2 (`/dev/ttyUSB0`) |
| Kamera | Intel RealSense D435b (bağlı ama aktif görevde kullanılmıyor) |

---

## Kurulum

```bash
# Bağımlılıklar
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-xacro \
     ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
     python3-numpy python3-serial

# Paketi derle
cd ~/ros2_ws
colcon build --packages-select omni_robot_pkg --symlink-install
source install/setup.bash
```

---

## Simülasyon (Gazebo Harmonic)

```bash
ros2 launch omni_robot_pkg simulation.launch.py

# Yaklaşık 9 saniye sonra, ayrı terminalde:
source ~/ros2_ws/install/setup.bash
ros2 service call /start_mission std_srvs/srv/Trigger
```

Seçenekler:

```bash
# GUI kapalı, RViz kapalı
ros2 launch omni_robot_pkg simulation.launch.py gui:=false rviz:=false
```

---

## Gerçek Robot

```bash
ros2 launch omni_robot_pkg robot_bringup.launch.py

# Yaklaşık 4 saniye sonra, ayrı terminalde:
source ~/ros2_ws/install/setup.bash
ros2 service call /start_mission std_srvs/srv/Trigger
```

RViz ile:

```bash
ros2 launch omni_robot_pkg robot_bringup.launch.py rviz:=true
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

Tüm parametreler `config/robot_params.yaml` dosyasındadır. Önemli olanlar:

| Parametre | Değer | Açıklama |
|---|---|---|
| `obstacle_safety_radius` | 0.50 m | Robota minimum yaklaşma mesafesi |
| `mission_stop_distance` | 0.50 m | Hedefe ne kadar uzakta durulsun |
| `lidar_min_range` | 0.30 m | Bu mesafenin altı filtrelenir (tekerler) |
| `max_linear_velocity` | 0.30 m/s | Maksimum hız |
| `dwa_min_clearance` | 0.38 m | DWA engel toleransı |

---

## Görev Durumları

```
IDLE → SCANNING → NAVIGATING_TO_TARGET → AT_TARGET → RETURNING_HOME → DONE
```

Görev durumunu izlemek için:

```bash
ros2 topic echo /mission_state
```

Acil durdurma:

```bash
ros2 service call /stop_mission std_srvs/srv/Trigger
```

---

## Port İzinleri (gerçek robot)

```bash
sudo usermod -aG dialout $USER
# Yeniden oturum açın veya:
sudo chmod 666 /dev/ttyAMA0 /dev/ttyAMA1 /dev/ttyUSB0
```
