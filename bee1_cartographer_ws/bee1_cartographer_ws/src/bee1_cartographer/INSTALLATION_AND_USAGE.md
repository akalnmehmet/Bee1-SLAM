# Beemobs Bee1 Cartographer SLAM & Navigation System
## Kurulum ve Kullanım Rehberi

### Sistem Gereksinimleri

- **İşletim Sistemi:** Ubuntu 22.04 LTS
- **ROS2:** Humble Hawksbill
- **Python:** 3.10+
- **Donanım:** ADVANTECH + RTX 3060 GPU
- **RAM:** Minimum 8GB (16GB önerilir)
- **Depolama:** 50GB boş alan

---

## 1. Sistem Kurulumu

### 1.1 ROS2 Humble Kurulumu

```bash
# Locale ayarları
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 repo ekle
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble kur
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 1.2 Gerekli Bağımlılıklar

```bash
sudo apt install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-velodyne \
    ros-humble-ublox \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs \
    python3-geojson \
    python3-pyproj \
    python3-colcon-common-extensions \
    python3-rosdep
```

### 1.3 Workspace Oluşturma

```bash
# Workspace dizini oluştur (kendi home directory'nizde)
cd ~
mkdir -p bee1_cartographer_ws && cd bee1_cartographer_ws && mkdir -p src/bee1_cartographer build install log maps && cd src/bee1_cartographer && mkdir -p include/bee1_cartographer src launch config/{cartographer,navigation,robot_localization,rviz,udev,network} urdf scripts docker test maps && touch include/bee1_cartographer/{geojson_parser,mission_executor,vehicle_controller,hardware_interface,system_monitor}.hpp src/{geojson_parser,mission_executor,vehicle_controller,hardware_interface,system_monitor}.cpp launch/bee1_{bringup,mapping,localization,navigation,hardware,full_system}.launch.py config/cartographer/bee1_{2d,3d}.lua config/navigation/{nav2_params.yaml,bt_navigator.xml} config/robot_localization/ekf.yaml config/rviz/bee1_{mapping,navigation}.rviz config/udev/99-bee1-devices.rules config/network/velodyne-network.yaml config/{bee1_params,VLP16db}.yaml urdf/bee1{.urdf.xacro,_sensors.urdf.xacro} scripts/{setup_bee1.sh,launch_all.sh,bee1_system.service} docker/entrypoint.sh test/test_basic_functionality.py maps/sample_track.yaml package.xml CMakeLists.txt README.md INSTALLATION_AND_USAGE.md LICENSE .gitignore Makefile Dockerfile docker-compose.yml && chmod +x scripts/{setup_bee1.sh,launch_all.sh} docker/entrypoint.sh

# rosdep başlat (ilk kez)
sudo rosdep init
rosdep update

# Environment source
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 2. Paket Kurulumu

### 2.1 Hızlı Kurulum (Otomatik)

```bash
# Kurulum scriptini indir ve çalıştır
wget https://raw.githubusercontent.com/akalnmehmet/Bee1-SLAM/main/scripts/setup_bee1.sh
chmod +x setup_bee1.sh
./setup_bee1.sh
```

### 2.2 Manuel Kurulum

```bash
# Workspace oluştur
mkdir -p ~/bee1_cartographer_ws/src
cd ~/bee1_cartographer_ws/src

# GitHub'dan repoyu clone et
git clone https://github.com/akalnmehmet/Bee1-SLAM.git bee1_cartographer

# Workspace'e geri dön
cd ~/bee1_cartographer_ws

# rosdep başlat (ilk kez çalıştırıyorsanız)
sudo rosdep init
rosdep update

# Bağımlılıkları yükle
rosdep install --from-paths src --ignore-src -r -y

# Environment source
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Paketi derle
colcon build --symlink-install

# Workspace setup
source install/setup.bash
echo "source ~/bee1_cartographer_ws/install/setup.bash" >> ~/.bashrc
```

### 2.3 Git Clone Seçenekleri

#### HTTPS ile Clone

```bash
# Public repo için
git clone https://github.com/akalnmehmet/Bee1-SLAM.git bee1_cartographer

# Private repo için (GitHub token gerekli)
git clone https://GITHUB_TOKEN@github.com/akalnmehmet/Bee1-SLAM.git bee1_cartographer
```

#### SSH ile Clone

```bash
# SSH key kurulumu gerekli
git clone git@github.com:akalnmehmet/Bee1-SLAM.git bee1_cartographer
```

#### Belirli Branch Clone

```bash
# Development branch
git clone -b development https://github.com/akalnmehmet/Bee1-SLAM.git bee1_cartographer

# Release tag
git clone -b v1.0.0 https://github.com/akalnmehmet/Bee1-SLAM.git bee1_cartographer
```

### 2.4 Dosya Yapısı (Oluşturulan)

```
~/bee1_cartographer_ws/
├── src/
│   └── bee1_cartographer/ (Bee1-SLAM repository)
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── README.md
│       ├── INSTALLATION_AND_USAGE.md
│       ├── LICENSE
│       ├── .gitignore
│       ├── Makefile
│       ├── Dockerfile
│       ├── docker-compose.yml
│       │
│       ├── include/
│       │   └── bee1_cartographer/
│       │       ├── geojson_parser.hpp
│       │       ├── mission_executor.hpp
│       │       ├── vehicle_controller.hpp
│       │       ├── hardware_interface.hpp
│       │       └── system_monitor.hpp
│       │
│       ├── src/
│       │   ├── geojson_parser.cpp
│       │   ├── mission_executor.cpp
│       │   ├── vehicle_controller.cpp
│       │   ├── hardware_interface.cpp
│       │   └── system_monitor.cpp
│       │
│       ├── launch/
│       │   ├── bee1_bringup.launch.py
│       │   ├── bee1_mapping.launch.py
│       │   ├── bee1_localization.launch.py
│       │   ├── bee1_navigation.launch.py
│       │   ├── bee1_hardware.launch.py
│       │   └── bee1_full_system.launch.py
│       │
│       ├── config/
│       │   ├── cartographer/
│       │   │   ├── bee1_2d.lua
│       │   │   └── bee1_3d.lua
│       │   ├── navigation/
│       │   │   ├── nav2_params.yaml
│       │   │   └── bt_navigator.xml
│       │   ├── robot_localization/
│       │   │   └── ekf.yaml
│       │   ├── rviz/
│       │   │   ├── bee1_mapping.rviz
│       │   │   └── bee1_navigation.rviz
│       │   ├── udev/
│       │   │   └── 99-bee1-devices.rules
│       │   ├── network/
│       │   │   └── velodyne-network.yaml
│       │   ├── bee1_params.yaml
│       │   └── VLP16db.yaml
│       │
│       ├── urdf/
│       │   ├── bee1.urdf.xacro
│       │   └── bee1_sensors.urdf.xacro
│       │
│       ├── scripts/
│       │   ├── setup_bee1.sh
│       │   ├── launch_all.sh
│       │   └── bee1_system.service
│       │
│       ├── docker/
│       │   └── entrypoint.sh
│       │
│       ├── test/
│       │   └── test_basic_functionality.py
│       │
│       └── maps/
│           └── sample_track.yaml
│
├── build/ (otomatik oluşur)
├── install/ (otomatik oluşur)
├── log/ (otomatik oluşur)
└── maps/ (kullanıcı haritaları)
```

### 2.5 Derleme

```bash
cd ~/bee1_cartographer_ws

# Bağımlılıkları kur
rosdep install --from-paths src --ignore-src -r -y

# Paketi derle
colcon build --symlink-install

# Environment setup
source install/setup.bash
echo "source ~/bee1_cartographer_ws/install/setup.bash" >> ~/.bashrc
```

---

## 3. Çalıştırma Talimatları

### 3.1 Tam Sistem Başlatma

```bash
# Terminal 1: Tüm sistemi başlat
source ~/bee1_cartographer_ws/install/setup.bash
ros2 launch bee1_cartographer bee1_full_system.launch.py
```

### 3.2 Hardware Interface Başlatma

```bash
# Terminal 1: Hardware başlat
ros2 launch bee1_cartographer bee1_hardware.launch.py

# Terminal 2: Sistem durumu kontrol
ros2 run bee1_cartographer system_monitor
```

### 3.3 Mapping Modu (İlk Harita Oluşturma)

```bash
# Terminal 1: Temel sistem başlat
ros2 launch bee1_cartographer bee1_bringup.launch.py

# Terminal 2: Mapping başlat
ros2 launch bee1_cartographer bee1_mapping.launch.py

# Terminal 3: Manuel kontrol (opsiyonel)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Haritayı kaydet
mkdir -p ~/bee1_cartographer_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/bee1_cartographer_ws/maps/my_map
```

### 3.4 Localization Modu

```bash
# Terminal 1: Temel sistem
ros2 launch bee1_cartographer bee1_bringup.launch.py

# Terminal 2: Localization
ros2 launch bee1_cartographer bee1_localization.launch.py map:=~/bee1_cartographer_ws/maps/my_map.yaml
```

### 3.5 Navigation Modu (Otonom Sürüş)

```bash
# Terminal 1: Navigation başlat
ros2 launch bee1_cartographer bee1_navigation.launch.py map:=~/bee1_cartographer_ws/maps/my_map.yaml

# Terminal 2: Mission executor başlat
ros2 run bee1_cartographer mission_executor

# Terminal 3: Vehicle controller başlat
ros2 run bee1_cartographer vehicle_controller
```

### 3.6 Manuel Görev Başlatma

```bash
# GeoJSON parser başlat
ros2 run bee1_cartographer geojson_parser

# Mission status kontrol
ros2 topic echo /mission_status

# Görev başlat
ros2 service call /mission_executor/start_mission std_srvs/srv/Empty
```

---

## 4. Docker ile Çalıştırma

### 4.1 Docker Build

```bash
cd ~/bee1_cartographer_ws/src/bee1_cartographer

# Docker image oluştur
docker build -t bee1_cartographer .

# Docker Compose ile çalıştır
docker-compose up -d
```

### 4.2 Container Yönetimi

```bash
# Container durumu kontrol
docker ps

# Container logları
docker logs bee1_cartographer

# Container'a bağlan
docker exec -it bee1_cartographer bash
```

---

## 5. Sistem Monitoring

### 5.1 Temel Kontroller

```bash
# ROS2 node listesi
ros2 node list

# Bee1 özel nodeları kontrol
ros2 node info /geojson_parser
ros2 node info /mission_executor
ros2 node info /vehicle_controller
ros2 node info /hardware_interface
ros2 node info /system_monitor

# Topic listesi
ros2 topic list

# Transform tree
ros2 run tf2_tools view_frames

# TF kontrolü
ros2 run tf2_ros tf2_echo map base_link
```

### 5.2 Sensör Kontrolleri

```bash
# GPS durumu
ros2 topic echo /gps/fix

# IMU durumu
ros2 topic echo /imu/data

# LiDAR durumu
ros2 topic echo /velodyne_points

# EKF çıktısı
ros2 topic echo /odometry/filtered

# Hardware diagnostics
ros2 topic echo /hardware/diagnostics
```

### 5.3 Navigation ve Mission Kontrolleri

```bash
# Mission durumu
ros2 topic echo /mission_status

# Vehicle kontrol durumu
ros2 topic echo /vehicle/diagnostics

# Navigation goals
ros2 topic echo /goal_pose

# Path planlama
ros2 topic echo /plan

# GeoJSON waypoints
ros2 topic echo /waypoints
```

---

## 6. Konfigürasyon Dosyaları

### 6.1 Ana Parametre Dosyası

```bash
# bee1_params.yaml düzenle
nano ~/bee1_cartographer_ws/src/bee1_cartographer/config/bee1_params.yaml
```

### 6.2 Network Konfigürasyonu

```bash
# Velodyne network ayarları
sudo nano ~/bee1_cartographer_ws/src/bee1_cartographer/config/network/velodyne-network.yaml

# Udev rules kur
sudo cp ~/bee1_cartographer_ws/src/bee1_cartographer/config/udev/99-bee1-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

### 6.3 VLP16 Konfigürasyonu

```bash
# VLP16 parametreleri düzenle
nano ~/bee1_cartographer_ws/src/bee1_cartographer/config/VLP16db.yaml
```

---

## 7. Troubleshooting

### 7.1 Hardware Interface Hataları

```bash
# Hardware interface restart
ros2 lifecycle set /hardware_interface deactivate
ros2 lifecycle set /hardware_interface activate

# System monitor logları
ros2 run bee1_cartographer system_monitor --ros-args --log-level DEBUG
```

### 7.2 Mission Executor Problemleri

```bash
# Mission executor reset
ros2 service call /mission_executor/reset std_srvs/srv/Empty

# GeoJSON parser restart
ros2 service call /geojson_parser/reload std_srvs/srv/Empty
```

### 7.3 Vehicle Controller Sorunları

```bash
# Vehicle controller diagnostics
ros2 topic echo /vehicle/diagnostics

# Emergency stop kontrol
ros2 topic echo /emergency_stop

# Manual override kontrol
ros2 topic echo /manual_override
```

### 7.4 Transform Hataları

```bash
# TF tree kontrol
ros2 run tf2_tools view_frames

# TF echo test
ros2 run tf2_ros tf2_echo map base_link

# Robot localization restart
ros2 lifecycle set /ekf_filter_node deactivate
ros2 lifecycle set /ekf_filter_node activate
```

---

## 8. Geliştirme ve Test

### 8.1 Test Çalıştırma

```bash
cd ~/bee1_cartographer_ws

# Temel fonksiyonalite testi
python3 src/bee1_cartographer/test/test_basic_functionality.py

# Colcon test
colcon test --packages-select bee1_cartographer
```

### 8.2 Makefile Kullanımı

```bash
cd ~/bee1_cartographer_ws/src/bee1_cartographer

# Build
make build

# Clean
make clean

# Install
make install

# Test
make test
```

### 8.3 Script Çalıştırma

```bash
# Setup script
./scripts/setup_bee1.sh

# Launch all script
./scripts/launch_all.sh

# System service
sudo systemctl enable ~/bee1_cartographer_ws/src/bee1_cartographer/scripts/bee1_system.service
sudo systemctl start bee1_system
```

---

## 9. Performance Optimizasyonu

### 9.1 CPU Optimizasyonu

```bash
# CPU governor performance mode
sudo cpufreq-set -g performance

# Real-time priority (dikkatli kullanın)
sudo echo 'kernel.sched_rt_runtime_us = -1' >> /etc/sysctl.conf

# Network optimizasyonu
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=134217728
```

### 9.2 GPU Kullanımı

```bash
# CUDA environment
export CUDA_VISIBLE_DEVICES=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# GPU monitoring
nvidia-smi
```

### 9.3 ROS2 DDS Optimizasyonu

```bash
# Cyclone DDS kullan (performans için)
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# QoS profilleri optimize et
export ROS_SECURITY_KEYSTORE_CONFIG_FILE=~/keystore_config.yaml
```

---

## 10. Güvenlik Kontrolleri

### 10.1 Emergency Stop

```bash
# Emergency stop aktif et
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once

# Emergency stop deaktif et
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" --once

# Hardware interface emergency status
ros2 topic echo /hardware/emergency_status
```

### 10.2 Hız Limiti Test

```bash
# Maksimum hız test (sistem 15 m/s ile sınırlamalı)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 20.0}}" --once

# Vehicle controller hız kontrol
ros2 topic echo /vehicle/diagnostics

# System monitor uyarıları
ros2 topic echo /system/warnings
```

### 10.3 Manuel Override

```bash
# Manuel kontrol aktif
ros2 topic pub /manual_override std_msgs/msg/Bool "{data: true}" --once

# Vehicle controller override durumu
ros2 topic echo /vehicle/manual_status
```

---

## 11. Bakım ve Güncellemeler

### 11.1 Paket Güncelleme

```bash
# ROS2 paketleri güncelle
sudo apt update
sudo apt upgrade ros-humble-*

# Repository güncelle
cd ~/bee1_cartographer_ws/src/bee1_cartographer
git pull origin main

# Workspace yeniden derle
cd ~/bee1_cartographer_ws
colcon build --symlink-install
```

### 11.2 Konfigürasyon Backup

```bash
# Tüm konfigürasyonları yedekle
tar -czf bee1_config_backup_$(date +%Y%m%d).tar.gz ~/bee1_cartographer_ws/src/bee1_cartographer/config/

# Haritaları yedekle
cp -r ~/bee1_cartographer_ws/maps ~/bee1_maps_backup_$(date +%Y%m%d)/

# Scripts yedekle
cp -r ~/bee1_cartographer_ws/src/bee1_cartographer/scripts ~/bee1_scripts_backup_$(date +%Y%m%d)/
```

---

## 12. Sıkça Sorulan Sorular

**S: Workspace nerede oluşturulmalı?**
C: `~/bee1_cartographer_ws` (kendi home directory'nizde)

**S: Repository nasıl indirilir?**
C: `git clone https://github.com/akalnmehmet/Bee1-SLAM.git bee1_cartographer`

**S: Hangi launch dosyası ne için kullanılır?**
C: 
- `bee1_bringup.launch.py`: Temel sistem
- `bee1_hardware.launch.py`: Hardware interface
- `bee1_mapping.launch.py`: Harita oluşturma
- `bee1_localization.launch.py`: Lokalizasyon
- `bee1_navigation.launch.py`: Navigasyon
- `bee1_full_system.launch.py`: Tüm sistem

**S: Hardware interface nasıl çalışır?**
C: `hardware_interface.cpp` sensör verilerini işler ve `system_monitor.cpp` sistem durumunu takip eder.

**S: Mission executor neden çalışmıyor?**
C: GeoJSON parser'ın waypoint'leri doğru yüklediğinden emin olun.

**S: Docker kullanmalı mıyım?**
C: Geliştirme için local, production için Docker önerilir.

**S: Güncelleme nasıl yapılır?**
C: `cd ~/bee1_cartographer_ws/src/bee1_cartographer && git pull origin main`

**S: Repository hangi branch'i kullanmalı?**
C: Ana geliştirme için `main` branch, test için `development` branch

---

## 📚 Documentation

- [GitHub Repository](https://github.com/akalnmehmet/Bee1-SLAM)
- [Installation Guide](https://github.com/akalnmehmet/Bee1-SLAM/blob/main/INSTALLATION_AND_USAGE.md)
- [API Reference](https://github.com/akalnmehmet/Bee1-SLAM/blob/main/docs/api.md)
- [Configuration Guide](https://github.com/akalnmehmet/Bee1-SLAM/blob/main/docs/configuration.md)
- [Hardware Setup](https://github.com/akalnmehmet/Bee1-SLAM/blob/main/docs/hardware.md)

## 🤝 Contributing

1. Fork the repository: [https://github.com/akalnmehmet/Bee1-SLAM/fork](https://github.com/akalnmehmet/Bee1-SLAM/fork)
2. Create feature branch: `git checkout -b feature/new-feature`
3. Commit changes: `git commit -am 'Add new feature'`
4. Push to branch: `git push origin feature/new-feature`
5. Submit pull request: [https://github.com/akalnmehmet/Bee1-SLAM/pulls](https://github.com/akalnmehmet/Bee1-SLAM/pulls)

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/akalnmehmet/Bee1-SLAM/issues)
- **Discussions**: [GitHub Discussions](https://github.com/akalnmehmet/Bee1-SLAM/discussions)
- **Author**: Mehmet AKALIN
- **Email**: mehmet@akalnmehmet.com

---

**🚗 Drive Autonomous, Drive Safe!** 

*Built with ❤️ for autonomous racing by Mehmet AKALIN*