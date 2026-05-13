# AisleBot

Asymmetric Mecanum Omnidirectional Robot for Narrow-Aisle Warehouse Navigation  
**Aritra Das (25D0074) | IIT Bombay | Prof. Ambarish Kunwar**

---

## Fresh Pi — one command

```bash
bash <(curl -sSL https://raw.githubusercontent.com/AritraD11/Aislebot/main/install.sh)
```

Installs everything: ROS2 Jazzy, Nav2, SLAM, all Python packages, builds the workspace, configures udev, systemd, and .bashrc. Takes ~25 minutes on a fresh Pi 5.

---

## Exact repo structure to maintain

```
aislebot/
│
├── install.sh                         ← one-click installer (this is the magic file)
├── README.md
│
├── system/                            ← system config files, copied from your Pi
│   ├── 99-aislebot.rules              ← /etc/udev/rules.d/  (from your Pi)
│   ├── aislebot.service               ← /etc/systemd/system/ (from your Pi)
│   └── start_aislebot.sh              ← ~/start_aislebot.sh  (from your Pi)
│
└── src/                               ← your ROS2 workspace src/, exactly as-is
    ├── mecanum_robot/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── resource/
    │   │   └── mecanum_robot          ← ament index marker (empty file)
    │   ├── mecanum_robot/
    │   │   ├── __init__.py
    │   │   ├── esp32_bridge.py
    │   │   ├── arm_bridge.py
    │   │   ├── phone_dashboard.py
    │   │   ├── mecanum_teleop_asymmetric.py
    │   │   ├── odometry_publisher.py
    │   │   ├── lcd_display.py
    │   │   ├── joy_to_aislebot.py
    │   │   ├── keyboard_teleop.py
    │   │   └── gazebo_bridge.py
    │   └── launch/
    │       ├── aislebot_full.launch.py
    │       ├── hardware.launch.py
    │       └── simulation.launch.py
    │
    └── mecanum_navigation/
        ├── package.xml
        ├── setup.py
        ├── resource/
        │   └── mecanum_navigation     ← ament index marker (empty file)
        ├── mecanum_navigation/
        │   └── __init__.py
        ├── launch/
        │   ├── slam.launch.py
        │   └── navigation.launch.py
        └── config/
            ├── ekf_params.yaml
            ├── nav2_params.yaml
            └── slam_params.yaml
```

---

## How to set it up (one time, on your current Pi)

```bash
# 1. Create the repo structure on your Pi
mkdir -p ~/aislebot/system
mkdir -p ~/aislebot/src

# 2. Copy system files
cp /etc/udev/rules.d/99-aislebot.rules  ~/aislebot/system/
cp /etc/systemd/system/aislebot.service ~/aislebot/system/
cp ~/start_aislebot.sh                  ~/aislebot/system/

# 3. Copy ROS2 source
cp -r ~/ros2_ws/src/mecanum_robot       ~/aislebot/src/
cp -r ~/ros2_ws/src/mecanum_navigation  ~/aislebot/src/

# 4. Add install.sh and README.md to ~/aislebot/
# (copy the files from this conversation)

# 5. Push to GitHub
cd ~/aislebot
git init
git add -A
git commit -m "Initial AisleBot setup"
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/aislebot.git
git push -u origin main
```

---

## Updating the repo when you change code

```bash
cd ~/aislebot
cp ~/ros2_ws/src/mecanum_robot/mecanum_robot/*.py src/mecanum_robot/mecanum_robot/
cp ~/ros2_ws/src/mecanum_robot/launch/*.py        src/mecanum_robot/launch/
git add -A
git commit -m "update: <what you changed>"
git push
```

---

## Hardware

| Component | Detail |
|---|---|
| Compute | Raspberry Pi 5, Ubuntu 24.04 LTS, ROS2 Jazzy |
| Drive controller | ESP32-WROOM-32 → `/dev/esp32` @ 921600 baud |
| Arm controller | Arduino Mega 2560 → `/dev/mega` @ 115200 baud |
| Motors | Rhino RMCS-2086 (24V, 60 RPM, 1:47, 93132 CPR) |
| Drivers | 2× Cytron MDD20A |
| Wheels | DekuPro 6-inch SR Mecanum (radius 0.0762 m) |
| Geometry | K_outer = 0.5607 m (FR, RL) · K_inner = 0.4907 m (FL, RR) |
| Arm | 2× NEMA23 (TB6600) + NEMA34 linear (BH-MSD-6A-W) |
| Sensors | RPLiDAR A1 (planned) · BNO055 IMU (planned) · 16×2 I2C LCD |

## Firmware (stored separately in Google Drive)

| File | Target | Baud |
|---|---|---|
| `aislebot_esp32_v2.ino` | ESP32-WROOM-32 | 921600 |
| `aislebot_arm_v7.ino` | Arduino Mega 2560 | 115200 |