# 🐝 Bee1-SLAM - Kurulum ve Kullanım Rehberi

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![License](https://img.shields.io/badge/License-Apache%202.0-green)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2022.04-orange)
![Docker](https://img.shields.io/badge/Docker-Supported-blue)

**🚗 ROS2 SLAM & Navigation system for Bee1 autonomous vehicle**

## 📋 **Sistem Gereksinimleri**

### Minimum Gereksinimler
- **İşletim Sistemi:** Ubuntu 22.04 LTS
- **ROS2:** Humble Hawksbill
- **Python:** 3.10+
- **RAM:** Minimum 8GB (16GB önerilir)
- **Depolama:** 50GB boş alan
- **Network:** Ethernet port (LiDAR için)

### Önerilen Donanım
- **İşlemci:** Intel i7 veya AMD Ryzen 7
- **GPU:** NVIDIA RTX 3060 veya üzeri (CUDA desteği)
- **RAM:** 16GB DDR4
- **SSD:** 100GB+ NVMe SSD
- **Donanım:** ADVANTECH endüstriyel bilgisayar

---

## 1. 🚀 Hızlı Başlangıç (Önerilen)

### 1.1 Otomatik Kurulum

```bash
# Kurulum scriptini indir ve çalıştır
curl -fsSL https://raw.githubusercontent.com/akalnmehmet/Bee1-SLAM/main/scripts/setup_bee1.sh -o setup_bee1.sh
chmod +x setup_bee1.sh
./setup_bee1.sh
```

Bu script otomatik olarak:
- ✅ ROS2 Humble kurulumu
- ✅ Gerekli tüm bağımlılıkları yükleme
- ✅ Workspace oluşturma
- ✅ Repository clone etme
- ✅ Derleme ve konfigürasyon

### 1.2 Docker ile Hızlı Başlangıç (Production)

```bash
# Repository clone
git clone https://github.com/akalnmehmet/Bee1-SLAM.git
cd Bee1-SLAM

# Docker Compose ile çalıştır
docker-compose up -d

# Container status kontrolü
docker-compose ps
```

---

## 2. 🔧 Manuel Kurulum (Geliştirici)

### 2.1 ROS2 Humble Kurulumu

```bash
# Locale ayarları
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 GPG key ve repository
sudo apt install software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble kurulum
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 2.2 Bee1-SLAM Bağımlılıkları

```bash
# Ana ROS2 paketleri
sudo apt install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-velodyne \
    ros-humble-velodyne-driver \
    ros-humble-velodyne-pointcloud \
    ros-humble-ublox \
    ros-humble-ublox-gps \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rqt-graph \
    ros-humble-rqt-robot-steering

# Python bağımlılıkları
sudo apt install -y \
    python3-geojson \
    python3-pyproj \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip

# Ek sistem araçları
sudo apt install -y \
    git \
    wget \
    build-essential \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev

# Python paketleri
pip3 install --user \
    numpy \
    scipy \
    matplotlib \
    pandas \
    pyyaml
```

### 2.3 Workspace Oluşturma ve Repository Clone

```bash
# Workspace dizini oluştur
mkdir -p ~/bee1_cartographer_ws/src
cd ~/bee1_cartographer_ws

# rosdep başlat (ilk kez)
sudo rosdep init
rosdep update

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Repository clone
cd src
git clone https://github.com/akalnmehmet/Bee1-SLAM.git bee1_cartographer

# Bağımlılıkları yükle
cd ~/bee1_cartographer_ws
rosdep install --from-paths src --ignore-src -r -y

# Paketi derle
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Workspace environment
source install/setup.bash
echo "source ~/bee1_cartographer_ws/install/setup.bash" >> ~/.bashrc
```

### 2.4 Dosya Yapısı Doğrulama

Kurulum sonrası dosya yapısını kontrol edin:

```bash
cd ~/bee1_cartographer_ws/src/bee1_cartographer

# Build sistemi
make build          # colcon build --symlink-install
make clean          # build/ install/ log/ temizle
make install        # Build + install
make test           # Testleri çalıştır

# Development
make format         # Code formatting
make lint           # Code linting
make docs           # Documentation oluştur

# Docker
make docker-build   # Docker image build
make docker-run     # Docker container çalıştır
make docker-clean   # Docker cleanup

# System
make setup          # Sistem kurulumu
make deps           # Dependencies yükle
make udev           # UDEV rules kur
```

---

## 12. 🚀 Production Deployment

### 12.1 Systemd Service Kurulumu

```bash
# Service dosyasını kopyala
sudo cp ~/bee1_cartographer_ws/src/bee1_cartographer/scripts/bee1_system.service /etc/systemd/system/

# Service dosyasını düzenle (kullanıcı yolu ayarla)
sudo nano /etc/systemd/system/bee1_system.service

# Service'i etkinleştir
sudo systemctl daemon-reload
sudo systemctl enable bee1_system.service

# Service'i başlat
sudo systemctl start bee1_system.service

# Service durumu
sudo systemctl status bee1_system.service

# Logları kontrol et
sudo journalctl -u bee1_system.service -f
```

Örnek service dosyası:
```ini
[Unit]
Description=Bee1 SLAM & Navigation System
After=network.target
Wants=network.target

[Service]
Type=exec
User=bee1user
Group=bee1user
WorkingDirectory=/home/bee1user/bee1_cartographer_ws
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedx_cpp"
ExecStartPre=/bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch bee1_cartographer bee1_full_system.launch.py"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### 12.2 Auto-Start Konfigürasyonu

```bash
# Boot time'da otomatik başlatma
sudo systemctl enable bee1_system.service

# Network bağlantısı sonrası başlatma
sudo nano /etc/systemd/system/bee1_system.service
# After=network-online.target ekle

# Kullanıcı login'i sonrası başlatma
systemctl --user enable bee1_system.service
```

### 12.3 Monitoring ve Logging

```bash
# Centralized logging
sudo mkdir -p /var/log/bee1
sudo chown $USER:$USER /var/log/bee1

# Log rotation konfigürasyonu
sudo nano /etc/logrotate.d/bee1
```

Logrotate konfigürasyonu:
```
/var/log/bee1/*.log {
    daily
    rotate 7
    compress
    delaycompress
    copytruncate
    create 644 bee1user bee1user
}
```

### 12.4 Performance Optimization

```bash
# CPU governor performance mode
echo 'performance' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Network buffer optimization
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=134217728
sudo sysctl -w net.core.wmem_max=134217728
sudo sysctl -w net.core.wmem_default=134217728

# Real-time priority (dikkatli kullanın!)
sudo echo 'kernel.sched_rt_runtime_us = -1' >> /etc/sysctl.conf

# GPU memory optimization (NVIDIA)
sudo nvidia-persistenced --persistence-mode
```

---

## 13. 🔄 Güncelleme ve Bakım

### 13.1 Sistem Güncellemeleri

```bash
# ROS2 paketleri güncelle
sudo apt update
sudo apt upgrade ros-humble-*

# Python dependencies güncelle
pip3 install --upgrade --user -r requirements.txt

# System packages güncelle
sudo apt update && sudo apt upgrade
```

### 13.2 Repository Güncellemeleri

```bash
cd ~/bee1_cartographer_ws/src/bee1_cartographer

# Mevcut değişiklikleri yedekle
git stash push -m "Local changes backup $(date)"

# Latest changes'ı al
git pull origin main

# Conflicts varsa çöz
git status
git diff

# Workspace'i yeniden derle
cd ~/bee1_cartographer_ws
colcon build --symlink-install

# Stash'i geri getir (gerekirse)
git stash pop
```

### 13.3 Konfigürasyon Yedekleme

```bash
# Tam sistem yedekleme
cd ~/bee1_cartographer_ws
tar -czf bee1_backup_$(date +%Y%m%d_%H%M%S).tar.gz \
  src/bee1_cartographer/config/ \
  src/bee1_cartographer/scripts/ \
  maps/ \
  install/

# Sadece konfigürasyon yedekleme
tar -czf bee1_config_backup_$(date +%Y%m%d).tar.gz \
  src/bee1_cartographer/config/

# Maps yedekleme
cp -r maps/ bee1_maps_backup_$(date +%Y%m%d)/

# Cloud backup (opsiyonel)
rsync -avz bee1_backup_*.tar.gz user@backup-server:/backup/bee1/
```

### 13.4 Log Yönetimi

```bash
# Log dosyalarını temizle
sudo journalctl --vacuum-time=7d
sudo journalctl --vacuum-size=1G

# ROS2 log cleanup
rm -rf ~/.ros/log/*

# Custom log cleanup
find /var/log/bee1 -name "*.log" -mtime +7 -delete
```

---

## 14. 🔐 Güvenlik ve Performans

### 14.1 Emergency Procedures

```bash
# Acil durum - sistem durdurma
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once

# Hardware emergency stop
ros2 service call /hardware_interface/emergency_stop std_srvs/srv/Empty

# Tüm ROS2 node'larını kapat
pkill -f ros2

# Systemd service'i durdur
sudo systemctl stop bee1_system.service
```

### 14.2 Safety Checks

```bash
# Sistem sağlık kontrolü
ros2 topic echo /diagnostics | grep -E "(ERROR|WARN)"

# Sensor health check
ros2 service call /system_monitor/health_check std_srvs/srv/Empty

# Vehicle safety status
ros2 topic echo /vehicle/safety_status --once

# Speed limiter test
ros2 param get /vehicle_controller max_linear_velocity
```

### 14.3 Security Hardening

```bash
# ROS2 security keystore (opsiyonel)
ros2 security create_keystore demo_keystore
ros2 security create_enclave demo_keystore /bee1_cartographer

# Network interface restriction
sudo iptables -A INPUT -p udp --dport 7400:7499 -s 192.168.1.0/24 -j ACCEPT
sudo iptables -A INPUT -p udp --dport 7400:7499 -j DROP

# User permissions
sudo adduser bee1operator
sudo usermod -a -G dialout,video bee1operator
```

---

## 15. 📚 Gelişmiş Kullanım

### 15.1 Custom Waypoint Generation

Python script ile waypoint oluşturma:
```python
#!/usr/bin/env python3
import json
import math

def generate_circular_waypoints(center_lat, center_lon, radius_m, num_points):
    """Generate circular waypoints"""
    waypoints = []
    
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        
        # Approximate lat/lon offset (not precise for large distances)
        lat_offset = radius_m * math.cos(angle) / 111319.5
        lon_offset = radius_m * math.sin(angle) / (111319.5 * math.cos(math.radians(center_lat)))
        
        waypoint = {
            "type": "Feature",
            "properties": {
                "name": f"waypoint_{i}",
                "local_x": radius_m * math.cos(angle),
                "local_y": radius_m * math.sin(angle),
                "heading": math.degrees(angle + math.pi/2),
                "speed": 10.0,
                "stop_duration": 0.5
            },
            "geometry": {
                "type": "Point",
                "coordinates": [center_lon + lon_offset, center_lat + lat_offset]
            }
        }
        waypoints.append(waypoint)
    
    return {
        "type": "FeatureCollection",
        "features": waypoints
    }

# Kullanım
circular_mission = generate_circular_waypoints(40.7903314, 29.50896659, 50, 8)
with open("circular_track.geojson", "w") as f:
    json.dump(circular_mission, f, indent=2)
```

### 15.2 Real-time Parameter Tuning

```bash
# Cartographer parametrelerini canlı değiştir
ros2 param set /cartographer_node trajectory_builder_2d.min_range 0.3
ros2 param set /cartographer_node trajectory_builder_2d.max_range 50.0

# Navigation parametreleri
ros2 param set /controller_server FollowPath.max_vel_x 10.0
ros2 param set /controller_server FollowPath.min_vel_x 2.0

# Safety parametreleri
ros2 param set /vehicle_controller emergency_brake_distance 5.0
```

### 15.3 Data Recording ve Analysis

```bash
# Tüm veriyi kaydet
ros2 bag record -a -o bee1_mission_$(date +%Y%m%d_%H%M%S)

# Specific topics kaydet
ros2 bag record -o navigation_data \
  /scan /odom /tf /tf_static /gps/fix /imu/data \
  /cmd_vel /mission_status /diagnostics

# Bag dosyasını analiz et
ros2 bag info navigation_data.db3
ros2 bag play navigation_data.db3

# Data export (CSV)
python3 scripts/bag_to_csv.py navigation_data.db3
```

---

## 16. 🆘 Sıkça Sorulan Sorular (FAQ)

### Kurulum ile İlgili

**S: Workspace nerede oluşturulmalı?**
A: `~/bee1_cartographer_ws` (home directory altında)

**S: Hangi ROS2 dağıtımı kullanılmalı?**
A: ROS2 Humble Hawksbill (Ubuntu 22.04 ile)

**S: Docker kullanmak zorunda mıyım?**
A: Hayır, native kurulum da destekleniyor. Docker sadece deployment kolaylığı için.

### Donanım ile İlgili

**S: Velodyne VLP16 bağlantı sorunu yaşıyorum.**
A: Static IP konfigürasyonunu kontrol edin (192.168.1.100/24) ve VLP16'ya ping atın (192.168.1.201).

**S: GPS fix alamıyorum.**
A: Açık alanda olduğunuzdan ve antenna bağlantısının doğru olduğundan emin olun. /gps/fix topic'ini kontrol edin.

**S: IMU verisi gelmiyor.**
A: UDEV rules kurulumunu kontrol edin ve kullanıcının dialout grubunda olduğundan emin olun.

### Yazılım ile İlgili

**S: Launch dosyaları ne işe yarar?**
A: 
- `bee1_bringup.launch.py`: Temel sistem (robot model, EKF, sensörler)
- `bee1_mapping.launch.py`: Haritalama modu
- `bee1_navigation.launch.py`: Navigasyon modu
- `bee1_full_system.launch.py`: Tam otonom sistem

**S: Mission executor neden çalışmıyor?**
A: GeoJSON dosyasının format kontrolü yapın ve /mission_status topic'ini takip edin.

**S: Harita kalitesi düşük.**
A: Yavaş hareket edin, LiDAR temizliğini kontrol edin ve cartographer parametrelerini ayarlayın.

### Performance ile İlgili

**S: Sistem yavaş çalışıyor.**
A: CPU governor'ı performance mode'a alın, sensor frequency'lerini düşürün.

**S: Memory kullanımı çok yüksek.**
A: Cartographer resolution'ı artırın ve ROS2 daemon'ı restart edin.

### Güvenlik ile İlgili

**S: Emergency stop nasıl kullanılır?**
A: `ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once`

**S: Manuel override nasıl aktif edilir?**
A: `/manual_override` topic'ini true yapın veya joystick kullanın.

**S: Maximum hız nedir?**
A: Yazılım limiti 15 m/s (55 km/h), güvenlik için ayarlanabilir.

---

## 17. 📞 Destek ve İletişim

### 🐛 Problem Bildirimi

**GitHub Issues**: [https://github.com/akalnmehmet/Bee1-SLAM/issues](https://github.com/akalnmehmet/Bee1-SLAM/issues)

Issue oluştururken şunları ekleyin:
- Ubuntu ve ROS2 versiyonları
- Error log'ları
- Reproduce steps
- Hardware konfigürasyonu

### 💬 Community Support

**GitHub Discussions**: [https://github.com/akalnmehmet/Bee1-SLAM/discussions](https://github.com/akalnmehmet/Bee1-SLAM/discussions)

### 📧 Direct Contact

**Author**: Mehmet AKALIN  
**Email**: mehmetakalin660@gmail.com  
**LinkedIn**: [linkedin.com/in/mehmetakalin](https://linkedin.com/in/mehmetakalin)

### 🤝 Contributing

1. Fork repository: [https://github.com/akalnmehmet/Bee1-SLAM/fork](https://github.com/akalnmehmet/Bee1-SLAM/fork)
2. Feature branch oluştur: `git checkout -b feature/amazing-feature`
3. Changes commit et: `git commit -m 'Add amazing feature'`
4. Branch'i push et: `git push origin feature/amazing-feature`
5. Pull Request oluştur

### 📖 Additional Resources

- **ROS2 Documentation**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- **Cartographer**: [https://google-cartographer.readthedocs.io/](https://google-cartographer.readthedocs.io/)
- **Navigation2**: [https://navigation.ros.org/](https://navigation.ros.org/)
- **Velodyne Driver**: [https://github.com/ros-drivers/velodyne](https://github.com/ros-drivers/velodyne)

---

## 📈 Version History

### v1.0.0 (Current)
- ✅ Complete SLAM & Navigation system
- ✅ Production-ready code
- ✅ Docker support
- ✅ Comprehensive documentation
- ✅ Safety systems
- ✅ Mission planning

### v0.9.0 (Beta)
- ✅ Core functionality
- ✅ Basic navigation
- ✅ Sensor integration

### Roadmap v1.1.0
- 🔄 Multi-robot support
- 🔄 Advanced behavior trees
- 🔄 Machine learning integration
- 🔄 Cloud connectivity

---

**🚗 Drive Autonomous, Drive Safe!** 

*Built with ❤️ for autonomous racing and robotics research*

**© 2024 Mehmet AKALIN - Apache License 2.0**

---

## 📋 Quick Reference Card

### 🔧 Essential Commands
```bash
# Quick start
./scripts/setup_bee1.sh

# Build
colcon build --symlink-install

# Full system
ros2 launch bee1_cartographer bee1_full_system.launch.py

# Emergency stop
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once

# Status check
ros2 topic echo /system_status
```

### 📁 Important Paths
```
~/bee1_cartographer_ws/          # Main workspace
src/bee1_cartographer/           # Source code
maps/                           # Saved maps
install/setup.bash              # Environment setup
config/                         # Configuration files
```

### 🎯 Key Topics
```
/scan                   # LiDAR data
/gps/fix               # GPS position
/imu/data              # IMU data
/odom                  # Odometry
/cmd_vel               # Velocity commands
/mission_status        # Mission state
/emergency_stop        # Emergency stop
```

Bu kapsamlı rehber ile Bee1-SLAM sistemini başarıyla kurabilir ve kullanabilirsiniz! 🚀ographer
tree -L 3
```

Beklenen yapı:
```
bee1_cartographer/
├── 📄 package.xml
├── 📄 CMakeLists.txt
├── 📄 README.md
├── 📄 INSTALLATION_AND_USAGE.md
├── 📂 include/bee1_cartographer/
├── 📂 src/
├── 📂 launch/
├── 📂 config/
├── 📂 urdf/
├── 📂 scripts/
├── 📂 docker/
├── 📂 test/
└── 📂 maps/
```

---

## 3. 🛠️ Donanım Konfigürasyonu

### 3.1 Network Konfigürasyonu (Velodyne VLP16)

```bash
# Static IP konfigürasyonu
sudo nano /etc/netplan/01-network-manager-all.yaml
```

Şu konfigürasyonu ekleyin:
```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:  # veya ilgili ethernet interface
      dhcp4: false
      addresses:
        - 192.168.1.100/24
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

```bash
# Network değişikliklerini uygula
sudo netplan apply

# VLP16 bağlantısını test et (VLP16 IP: 192.168.1.201)
ping 192.168.1.201
```

### 3.2 UDEV Rules Kurulumu

```bash
# USB cihazlar için udev rules kur
sudo cp ~/bee1_cartographer_ws/src/bee1_cartographer/config/udev/99-bee1-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Kullanıcıyı dialout grubuna ekle
sudo usermod -a -G dialout $USER

# Logout/login gerekli veya:
newgrp dialout
```

### 3.3 CAN Bus Konfigürasyonu (Opsiyonel)

```bash
# CAN utils kurulum
sudo apt install can-utils

# CAN interface kurulum
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_bcm

# can0 interface yapılandırma
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# CAN bus test
cansend can0 123#DEADBEEF
candump can0
```

---

## 4. 🏃 Sistem Çalıştırma

### 4.1 Temel Sistem Başlatma

```bash
# Terminal 1: Hardware interface başlat
cd ~/bee1_cartographer_ws
source install/setup.bash
ros2 launch bee1_cartographer bee1_hardware.launch.py

# Terminal 2: Temel sistem (robot state, EKF, sensörler)
ros2 launch bee1_cartographer bee1_bringup.launch.py
```

### 4.2 Haritalama Modu (İlk Kullanım)

```bash
# Terminal 1: Temel sistem + hardware
ros2 launch bee1_cartographer bee1_hardware.launch.py

# Terminal 2: Haritalama başlat
ros2 launch bee1_cartographer bee1_mapping.launch.py

# Terminal 3: RViz ile görselleştirme (otomatik açılır)
# Manuel açmak için:
# rviz2 -d ~/bee1_cartographer_ws/src/bee1_cartographer/config/rviz/bee1_mapping.rviz

# Terminal 4: Manuel kontrol (harita oluşturmak için)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

#### Haritayı Kaydetme

```bash
# Harita kaydetme
mkdir -p ~/bee1_cartographer_ws/maps
cd ~/bee1_cartographer_ws/maps

# Cartographer'dan harita al
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$(pwd)/bee1_track.pbstream'}"

# Navigation2 için yaml harita oluştur
ros2 run nav2_map_server map_saver_cli -f bee1_track

# Dosya kontrolü
ls -la bee1_track.*
# Çıktı: bee1_track.pbstream, bee1_track.yaml, bee1_track.pgm
```

### 4.3 Lokalizasyon Modu

```bash
# Önceden kaydedilmiş harita ile lokalizasyon
ros2 launch bee1_cartographer bee1_localization.launch.py \
  map:=~/bee1_cartographer_ws/maps/bee1_track.yaml

# İlk pozisyon estimate (RViz'de "2D Pose Estimate" kullanın)
```

### 4.4 Navigasyon Modu (Otonom Sürüş)

```bash
# Terminal 1: Navigasyon sistemi başlat
ros2 launch bee1_cartographer bee1_navigation.launch.py \
  map:=~/bee1_cartographer_ws/maps/bee1_track.yaml

# Terminal 2: Mission executor başlat
ros2 run bee1_cartographer mission_executor

# Terminal 3: Vehicle controller başlat
ros2 run bee1_cartographer vehicle_controller

# Terminal 4: System monitor
ros2 run bee1_cartographer system_monitor
```

### 4.5 Tam Sistem (Production)

```bash
# Tek komutla tüm sistemi başlat
ros2 launch bee1_cartographer bee1_full_system.launch.py \
  map:=~/bee1_cartographer_ws/maps/bee1_track.yaml \
  use_sim_time:=false \
  autostart:=true

# Veya script ile
cd ~/bee1_cartographer_ws/src/bee1_cartographer/scripts
./launch_all.sh
```

---

## 5. 🗺️ Mission Planning

### 5.1 GeoJSON Waypoint Oluşturma

Örnek GeoJSON formatı:
```json
{
    "type": "FeatureCollection",
    "crs": {
        "type": "name",
        "properties": {
            "name": "EPSG:4326"
        }
    },
    "features": [
        {
            "type": "Feature",
            "properties": {
                "name": "start_point",
                "local_x": 0.0,
                "local_y": 0.0,
                "heading": 0.0,
                "speed": 5.0,
                "stop_duration": 0.0
            },
            "geometry": {
                "type": "Point",
                "coordinates": [29.50896659, 40.7903314]
            }
        },
        {
            "type": "Feature",
            "properties": {
                "name": "waypoint_1",
                "local_x": 50.0,
                "local_y": 0.0,
                "heading": 90.0,
                "speed": 10.0,
                "stop_duration": 2.0
            },
            "geometry": {
                "type": "Point",
                "coordinates": [29.50946659, 40.7903314]
            }
        },
        {
            "type": "Feature",
            "properties": {
                "name": "waypoint_2",
                "local_x": 50.0,
                "local_y": 50.0,
                "heading": 180.0,
                "speed": 15.0,
                "stop_duration": 1.0
            },
            "geometry": {
                "type": "Point",
                "coordinates": [29.50946659, 40.7908314]
            }
        },
        {
            "type": "Feature",
            "properties": {
                "name": "finish_point",
                "local_x": 0.0,
                "local_y": 50.0,
                "heading": 270.0,
                "speed": 5.0,
                "stop_duration": 5.0
            },
            "geometry": {
                "type": "Point",
                "coordinates": [29.50896659, 40.7908314]
            }
        }
    ]
}
```

### 5.2 Mission Yükleme ve Başlatma

```bash
# GeoJSON dosyasını kaydet
nano ~/bee1_cartographer_ws/maps/mission_track.geojson
# Yukarıdaki JSON içeriğini yapıştır

# Mission yükle
ros2 service call /geojson_parser/load_mission \
  bee1_interfaces/srv/LoadMission \
  "{mission_file: '/home/$USER/bee1_cartographer_ws/maps/mission_track.geojson'}"

# Mission durumunu kontrol et
ros2 topic echo /mission_status

# Mission başlat
ros2 service call /mission_executor/start_mission std_srvs/srv/Empty

# Mission durdur (emergency)
ros2 service call /mission_executor/stop_mission std_srvs/srv/Empty

# Mission sıfırla
ros2 service call /mission_executor/reset_mission std_srvs/srv/Empty
```

---

## 6. 🐳 Docker Kullanımı

### 6.1 Docker Image Build

```bash
cd ~/Bee1-SLAM

# Development image
docker build -t bee1-slam:dev .

# Production image
docker build -t bee1-slam:prod -f docker/Dockerfile.prod .

# Multi-architecture build
docker buildx build --platform linux/amd64,linux/arm64 -t bee1-slam:latest .
```

### 6.2 Development Container

```bash
# X11 forwarding ile GUI desteği
xhost +local:docker

# Development container çalıştır
docker run -it --rm \
  --privileged \
  --network host \
  --ipc host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/bee1_cartographer_ws:/workspace \
  -v /dev:/dev \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=0 \
  --name bee1-dev \
  bee1-slam:dev
```

### 6.3 Production Container

```bash
# Production container (daemon mode)
docker run -d \
  --restart unless-stopped \
  --privileged \
  --network host \
  --ipc host \
  -v /dev:/dev \
  -v $HOME/bee1_maps:/workspace/maps \
  -v $HOME/bee1_logs:/workspace/logs \
  -e ROS_DOMAIN_ID=0 \
  --name bee1-prod \
  bee1-slam:prod \
  ros2 launch bee1_cartographer bee1_full_system.launch.py

# Container durumu
docker ps
docker logs bee1-prod

# Container'a bağlan
docker exec -it bee1-prod bash
```

### 6.4 Docker Compose

```bash
cd ~/Bee1-SLAM

# Servisleri başlat
docker-compose up -d

# Specific service başlat
docker-compose up bee1-mapping

# Logları takip et
docker-compose logs -f

# Servisleri durdur
docker-compose down

# Volume'ları da sil
docker-compose down -v
```

---

## 7. 📊 Sistem Monitoring

### 7.1 Temel Durum Kontrolleri

```bash
# Tüm ROS2 node'larını listele
ros2 node list

# Bee1 özel node'ları kontrol
ros2 node list | grep bee1

# Node detayları
ros2 node info /geojson_parser
ros2 node info /mission_executor
ros2 node info /vehicle_controller
ros2 node info /hardware_interface
ros2 node info /system_monitor

# Topic listesi
ros2 topic list

# Service listesi
ros2 service list
```

### 7.2 Sensör Durumu Monitoring

```bash
# GPS durumu
ros2 topic echo /ublox_gps/fix --once
ros2 topic echo /ublox_gps/navstatus --once

# IMU durumu
ros2 topic echo /imu/data --once

# LiDAR durumu
ros2 topic echo /velodyne_points --once
ros2 topic hz /velodyne_points

# Velodyne diagnostics
ros2 topic echo /velodyne_driver/diagnostics

# EKF filtered odometry
ros2 topic echo /odometry/filtered --once

# GPS-EKF status
ros2 topic echo /diagnostics | grep -A 10 robot_localization
```

### 7.3 SLAM ve Navigation Monitoring

```bash
# Cartographer map data
ros2 topic echo /map --once
ros2 topic hz /scan

# Transform tree kontrol
ros2 run tf2_tools view_frames
# Çıktı: frames.pdf dosyası oluşur

# TF echo test
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link velodyne

# Navigation durumu
ros2 topic echo /amcl_pose --once
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /plan --once

# Mission status
ros2 topic echo /mission_status
ros2 topic echo /waypoints
```

### 7.4 Vehicle Control Monitoring

```bash
# Vehicle diagnostics
ros2 topic echo /vehicle/diagnostics

# Control commands
ros2 topic echo /cmd_vel

# Ackermann commands
ros2 topic echo /ackermann_cmd

# Emergency stop status
ros2 topic echo /emergency_stop

# Manual override status
ros2 topic echo /manual_override

# System health
ros2 topic echo /system/warnings
ros2 topic echo /system/errors
```

### 7.5 Performance Monitoring

```bash
# CPU ve Memory kullanımı
htop
sudo iotop

# ROS2 specific monitoring
ros2 run rqt_graph rqt_graph
ros2 run rqt_plot rqt_plot

# Network monitoring
sudo nethogs
iftop

# GPU monitoring (NVIDIA)
watch -n 1 nvidia-smi

# Disk I/O
iostat -x 1
```

---

## 8. 🔧 Konfigürasyon Düzenleme

### 8.1 Ana Parametre Dosyası

```bash
# Sistem parametrelerini düzenle
nano ~/bee1_cartographer_ws/src/bee1_cartographer/config/bee1_params.yaml
```

Önemli parametreler:
```yaml
bee1:
  # Vehicle specifications
  max_linear_velocity: 15.0    # m/s (55 km/h)
  max_angular_velocity: 1.0    # rad/s
  wheelbase: 1.86              # meters
  
  # Safety limits
  emergency_stop_enabled: true
  manual_override_enabled: true
  speed_limit_enabled: true
  
  # Sensor configuration
  use_gps: true
  use_imu: true
  use_lidar: true
  
  # Navigation
  goal_tolerance: 0.5          # meters
  path_lookahead: 2.0          # meters
```

### 8.2 Cartographer Ayarları

```bash
# 2D SLAM parametreleri düzenle
nano ~/bee1_cartographer_ws/src/bee1_cartographer/config/cartographer/bee1_2d.lua
```

### 8.3 Navigation2 Ayarları

```bash
# Nav2 controller parametreleri
nano ~/bee1_cartographer_ws/src/bee1_cartographer/config/navigation/nav2_params.yaml
```

### 8.4 Robot Localization (EKF) Ayarları

```bash
# GPS+IMU fusion parametreleri
nano ~/bee1_cartographer_ws/src/bee1_cartographer/config/robot_localization/ekf.yaml
```

### 8.5 Velodyne VLP16 Konfigürasyonu

```bash
# LiDAR kalibrasyonu
nano ~/bee1_cartographer_ws/src/bee1_cartographer/config/VLP16db.yaml

# Network ayarları
nano ~/bee1_cartographer_ws/src/bee1_cartographer/config/network/velodyne-network.yaml
```

---

## 9. 🐛 Troubleshooting

### 9.1 Kurulum Sorunları

**ROS2 kurulum hataları:**
```bash
# GPG key hatası
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

# Repository güncelleme
sudo apt update --fix-missing

# Broken packages
sudo apt --fix-broken install
```

**Dependencies hatası:**
```bash
# rosdep güncelleme
rosdep update --include-eol-distros

# Force install
rosdep install --from-paths src --ignore-src -r -y --skip-keys="geographic_msgs"
```

**Build hataları:**
```bash
# Clean build
cd ~/bee1_cartographer_ws
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Verbose build
colcon build --event-handlers console_direct+
```

### 9.2 Donanım Sorunları

**GPS bağlantı problemi:**
```bash
# USB device kontrol
lsusb | grep u-blox

# Serial port kontrol
sudo dmesg | grep tty

# GPS status test
ros2 topic echo /ublox_gps/fix --once

# GPS restart
sudo systemctl restart gpsd
```

**LiDAR bağlantı problemi:**
```bash
# Network interface kontrol
ip addr show

# VLP16 ping test
ping 192.168.1.201

# Port listening test
sudo netstat -tulpn | grep 2368

# Velodyne driver restart
ros2 lifecycle set /velodyne_driver deactivate
ros2 lifecycle set /velodyne_driver activate
```

**IMU bağlantı problemi:**
```bash
# USB cihaz kontrol
lsusb | grep Xsens

# Permission kontrol
ls -la /dev/ttyUSB*

# IMU test
ros2 topic echo /imu/data --once

# UDEV rules yeniden yükle
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 9.3 SLAM Sorunları

**Cartographer başlamıyor:**
```bash
# TF tree kontrol
ros2 run tf2_tools view_frames

# Scan data kontrol
ros2 topic echo /scan --once

# Cartographer restart
ros2 lifecycle set /cartographer_node deactivate
ros2 lifecycle set /cartographer_node activate
```

**Haritalama kalitesi düşük:**
```bash
# Scan frequency kontrol
ros2 topic hz /scan

# Odometry kontrol
ros2 topic echo /odom --once

# Cartographer status
ros2 topic echo /cartographer/trajectory_node_list
```

### 9.4 Navigation Sorunları

**AMCL lokalizasyon problemi:**
```bash
# Initial pose set
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}" --once

# AMCL restart
ros2 lifecycle set /amcl deactivate
ros2 lifecycle set /amcl activate

# Particle cloud kontrol
ros2 topic echo /particle_cloud --once
```

**Navigation goal ulaşamıyor:**
```bash
# Costmap temizle
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap

# Path planner restart
ros2 lifecycle set /planner_server deactivate
ros2 lifecycle set /planner_server activate

# Controller restart
ros2 lifecycle set /controller_server deactivate
ros2 lifecycle set /controller_server activate
```

### 9.5 Mission Execution Sorunları

**GeoJSON parser hatası:**
```bash
# JSON format kontrol
python3 -m json.tool ~/bee1_cartographer_ws/maps/mission_track.geojson

# Mission reload
ros2 service call /geojson_parser/reload std_srvs/srv/Empty

# Parser restart
ros2 run bee1_cartographer geojson_parser --ros-args --log-level DEBUG
```

**Vehicle control yanıt vermiyor:**
```bash
# Emergency stop kontrolü
ros2 topic echo /emergency_stop --once

# Manual override kontrolü
ros2 topic echo /manual_override --once

# Vehicle controller restart
ros2 run bee1_cartographer vehicle_controller --ros-args --log-level DEBUG

# CAN bus kontrol (eğer kullanılıyorsa)
candump can0
```

### 9.6 Performance Sorunları

**Yüksek CPU kullanımı:**
```bash
# ROS2 process monitoring
top -p $(pgrep -f ros2)

# Sensor frequency düşür
ros2 param set /velodyne_driver frequency 5.0

# Cartographer resolution artır
# bee1_2d.lua dosyasında resolution değerini 0.05'ten 0.1'e çıkar
```

**Memory leak:**
```bash
# Memory monitoring
watch -n 1 'free -h && ps aux --sort=-%mem | head -20'

# ROS2 daemon restart
ros2 daemon stop
ros2 daemon start

# System memory temizle
sudo sysctl -w vm.drop_caches=3
```

### 9.7 Network Sorunları

**ROS2 discovery problemi:**
```bash
# ROS_DOMAIN_ID kontrol
echo $ROS_DOMAIN_ID

# Network discovery test
ros2 multicast receive
ros2 multicast send

# DDS config
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI=file:///path/to/cyclonedx.xml
```

---

## 10. 🧪 Test ve Doğrulama

### 10.1 Birim Testleri

```bash
cd ~/bee1_cartographer_ws

# Python testleri çalıştır
python3 src/bee1_cartographer/test/test_basic_functionality.py

# ROS2 testleri
colcon test --packages-select bee1_cartographer
colcon test-result --verbose
```

### 10.2 Entegrasyon Testleri

```bash
# Sensor integration test
ros2 launch bee1_cartographer test_sensors.launch.py

# Navigation stack test
ros2 launch bee1_cartographer test_navigation.launch.py

# Full system test
ros2 launch bee1_cartographer test_full_system.launch.py
```

### 10.3 Simülasyon Testleri

```bash
# Gazebo simülasyonu
ros2 launch bee1_cartographer bee1_simulation.launch.py

# Simülasyon ile navigation test
ros2 launch bee1_cartographer bee1_sim_navigation.launch.py
```

### 10.4 Hardware-in-the-Loop Test

```bash
# Gerçek sensörler + simülasyon environment
ros2 launch bee1_cartographer bee1_hil_test.launch.py

# Performance benchmark
ros2 run bee1_cartographer performance_benchmark
```

---

## 11. 🔧 Makefile Kullanımı

```bash
cd ~/bee1_cartographer_ws/src/bee1_cart
