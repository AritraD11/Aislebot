#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════════╗
# ║  AisleBot — One-Click Fresh Install                                  ║
# ║  Aritra Das (25D0074) | IIT Bombay | Prof. Ambarish Kunwar          ║
# ║                                                                      ║
# ║  Tested on: Raspberry Pi 5, Ubuntu 24.04 LTS                        ║
# ║  Installs : ROS2 Jazzy + Nav2 + SLAM + all nodes + system config    ║
# ║                                                                      ║
# ║  Run on a FRESH Pi with ONE command:                                 ║
# ║    bash <(curl -sSL https://raw.githubusercontent.com/              ║
# ║           YOUR_GITHUB_USERNAME/aislebot/main/install.sh)            ║
# ║                                                                      ║
# ║  Or after cloning:                                                   ║
# ║    git clone https://github.com/YOUR_USERNAME/aislebot               ║
# ║    cd aislebot && bash install.sh                                    ║
# ╚══════════════════════════════════════════════════════════════════════╝

set -e

# ════════════════════════════════════════════════════════════════════════
#  !! EDIT THIS BEFORE PUSHING TO GITHUB !!
# ════════════════════════════════════════════════════════════════════════
GITHUB_USER="AritrD11"
GITHUB_REPO="Aislebot"
GITHUB_BRANCH="main"

# ════════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ════════════════════════════════════════════════════════════════════════
REPO_URL="https://github.com/${GITHUB_USER}/${GITHUB_REPO}.git"
ROS_DISTRO="jazzy"
ROS_WS="$HOME/ros2_ws"
INSTALL_LOG="$HOME/aislebot_install_$(date +%Y%m%d_%H%M%S).log"
CLONE_DIR="/tmp/aislebot_src"

# ════════════════════════════════════════════════════════════════════════
#  HELPERS
# ════════════════════════════════════════════════════════════════════════
G='\033[0;32m'; Y='\033[1;33m'; R='\033[0;31m'; C='\033[0;36m'; B='\033[1m'; N='\033[0m'
ok()   { echo -e "${G}  ✓  $*${N}"; }
warn() { echo -e "${Y}  ⚠  $*${N}"; }
die()  { echo -e "${R}  ✗  $*${N}"; exit 1; }
step() { echo -e "\n${C}${B}━━━  $*${N}"; }

add_line() { grep -qxF "$1" "$HOME/.bashrc" || echo "$1" >> "$HOME/.bashrc"; }

# ════════════════════════════════════════════════════════════════════════
#  BANNER
# ════════════════════════════════════════════════════════════════════════
clear
echo -e "${B}${C}"
echo "  ╔══════════════════════════════════════════════════════╗"
echo "  ║         AisleBot — Complete Fresh Install            ║"
echo "  ║   Pi 5  ·  Ubuntu 24.04  ·  ROS2 Jazzy  ·  Nav2    ║"
echo "  ╚══════════════════════════════════════════════════════╝"
echo -e "${N}"
echo "  GitHub  : ${REPO_URL}"
echo "  Log     : ${INSTALL_LOG}"
echo "  Started : $(date)"
echo ""
echo "  This takes 20-30 minutes on a fresh Pi."
echo "  Keep the Pi connected to the internet."
echo ""
read -rp "  Press ENTER to start (Ctrl-C to cancel) ..." _
echo "AisleBot Install — $(date)" > "$INSTALL_LOG"

# ════════════════════════════════════════════════════════════════════════
#  PRE-CHECKS
# ════════════════════════════════════════════════════════════════════════
step "Pre-flight checks"

[ "$(id -u)" = "0" ] && die "Do NOT run as root. Run as your normal user."
ok "Running as $(whoami)"

DISTRO=$(lsb_release -cs 2>/dev/null || echo "unknown")
[ "$DISTRO" = "noble" ] || die "Requires Ubuntu 24.04 (noble). Found: $DISTRO"
ok "Ubuntu 24.04 confirmed"

curl -sSf https://github.com > /dev/null 2>&1 || die "No internet. Check your connection."
ok "Internet reachable"

[ "$GITHUB_USER" = "YOUR_GITHUB_USERNAME" ] && \
    die "Set GITHUB_USER at the top of install.sh to your actual GitHub username."
ok "GitHub: ${GITHUB_USER}/${GITHUB_REPO}"

# ════════════════════════════════════════════════════════════════════════
#  STEP 1 — SYSTEM PACKAGES
# ════════════════════════════════════════════════════════════════════════
step "1 / 7  —  System update & base tools"

sudo apt-get update -q >> "$INSTALL_LOG" 2>&1
ok "apt update"

sudo apt-get install -y -q \
    git curl wget locales software-properties-common \
    python3-pip python3-dev build-essential \
    unzip zip i2c-tools \
    >> "$INSTALL_LOG" 2>&1
ok "Base tools installed"

sudo locale-gen en_US en_US.UTF-8 >> "$INSTALL_LOG" 2>&1
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 >> "$INSTALL_LOG" 2>&1
export LANG=en_US.UTF-8
ok "Locale set"

# ════════════════════════════════════════════════════════════════════════
#  STEP 2 — ROS2 JAZZY
# ════════════════════════════════════════════════════════════════════════
step "2 / 7  —  ROS2 Jazzy (longest step ~15 min)"

if command -v ros2 &>/dev/null; then
    ok "ROS2 already installed — skipping"
else
    sudo add-apt-repository universe -y >> "$INSTALL_LOG" 2>&1

    sudo curl -sSL \
        https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg >> "$INSTALL_LOG" 2>&1

    echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt-get update -q >> "$INSTALL_LOG" 2>&1
    ok "ROS2 apt repo added"

    echo "  Installing ros-jazzy-desktop (this is the slow part)..."
    sudo apt-get install -y -q ros-jazzy-desktop >> "$INSTALL_LOG" 2>&1
    ok "ros-jazzy-desktop installed"

    sudo apt-get install -y -q python3-colcon-common-extensions >> "$INSTALL_LOG" 2>&1
    ok "colcon installed"
fi

# Additional packages — exact set confirmed on your Pi
echo "  Installing ROS2 add-on packages..."
sudo apt-get install -y -q \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-joy \
    ros-jazzy-rplidar-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf-transformations \
    ros-jazzy-rviz2 \
    >> "$INSTALL_LOG" 2>&1
ok "All ROS2 add-on packages installed"

source /opt/ros/${ROS_DISTRO}/setup.bash

# ════════════════════════════════════════════════════════════════════════
#  STEP 3 — PYTHON PACKAGES
# ════════════════════════════════════════════════════════════════════════
step "3 / 7  —  Python packages"

pip3 install --break-system-packages --quiet \
    "fastapi==0.136.1" \
    "uvicorn==0.46.0" \
    "pyserial==3.5" \
    "RPLCD==1.4.0" \
    "pandas==3.0.0" \
    "numpy" \
    "openpyxl" \
    >> "$INSTALL_LOG" 2>&1
ok "fastapi · uvicorn · pyserial · RPLCD · pandas · numpy · openpyxl"

# ════════════════════════════════════════════════════════════════════════
#  STEP 4 — CLONE REPO
# ════════════════════════════════════════════════════════════════════════
step "4 / 7  —  Cloning AisleBot repo"

rm -rf "$CLONE_DIR"
git clone --branch "$GITHUB_BRANCH" "$REPO_URL" "$CLONE_DIR" >> "$INSTALL_LOG" 2>&1
ok "Cloned: ${REPO_URL}"

# ════════════════════════════════════════════════════════════════════════
#  STEP 5 — BUILD WORKSPACE
# ════════════════════════════════════════════════════════════════════════
step "5 / 7  —  Building ROS2 workspace"

mkdir -p "${ROS_WS}/src"

for pkg in "${CLONE_DIR}/src"/*/; do
    PKG_NAME=$(basename "$pkg")
    DEST="${ROS_WS}/src/${PKG_NAME}"
    [ -d "$DEST" ] && { warn "${PKG_NAME} exists — overwriting"; rm -rf "$DEST"; }
    cp -r "$pkg" "$DEST"
    ok "Staged: ${PKG_NAME}"
done

cd "$ROS_WS"
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    >> "$INSTALL_LOG" 2>&1
ok "colcon build complete"
source "${ROS_WS}/install/setup.bash"

# ════════════════════════════════════════════════════════════════════════
#  STEP 6 — SYSTEM FILES
# ════════════════════════════════════════════════════════════════════════
step "6 / 7  —  System configuration"

SYS="${CLONE_DIR}/system"

# udev rules
if [ -f "${SYS}/99-aislebot.rules" ]; then
    sudo cp "${SYS}/99-aislebot.rules" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ok "udev: /dev/esp32 (CP2102 10c4:ea60) · /dev/mega (CH340 1a86:7523)"
else
    warn "99-aislebot.rules not found in repo/system/ — add it!"
fi

# start script
if [ -f "${SYS}/start_aislebot.sh" ]; then
    sed "s|/home/aritra|/home/$(whoami)|g" \
        "${SYS}/start_aislebot.sh" > "$HOME/start_aislebot.sh"
    chmod +x "$HOME/start_aislebot.sh"
    ok "start_aislebot.sh → ~/start_aislebot.sh"
else
    warn "start_aislebot.sh not found in repo/system/"
fi

# systemd service
if [ -f "${SYS}/aislebot.service" ]; then
    sed "s|aritra|$(whoami)|g" "${SYS}/aislebot.service" \
        | sudo tee /etc/systemd/system/aislebot.service > /dev/null
    sudo systemctl daemon-reload
    ok "aislebot.service installed"
    echo ""
    read -rp "  Auto-start AisleBot on boot? [y/N] " yn
    if [[ "$yn" =~ ^[Yy]$ ]]; then
        sudo systemctl enable aislebot.service
        ok "Autostart ENABLED"
    else
        ok "Autostart skipped  (enable later: sudo systemctl enable aislebot)"
    fi
else
    warn "aislebot.service not found in repo/system/"
fi

# Groups
if ! groups | grep -q dialout; then
    sudo usermod -aG dialout "$(whoami)" >> "$INSTALL_LOG" 2>&1
    ok "Added $(whoami) to dialout (serial access)"
fi
if ! groups | grep -q i2c; then
    sudo usermod -aG i2c "$(whoami)" >> "$INSTALL_LOG" 2>&1
    ok "Added $(whoami) to i2c (LCD)"
fi

# ════════════════════════════════════════════════════════════════════════
#  STEP 7 — .bashrc
# ════════════════════════════════════════════════════════════════════════
step "7 / 7  —  Shell environment"

add_line ""
add_line "# ── AisleBot ROS2 Environment ───────────────────────────────"
add_line "source /opt/ros/${ROS_DISTRO}/setup.bash"
add_line "source ${ROS_WS}/install/setup.bash"
add_line "export ROS_DOMAIN_ID=42"
add_line "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
add_line "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>'"
add_line ""
add_line "# ── AisleBot Aliases ────────────────────────────────────────"
add_line "alias ab='ros2 launch mecanum_robot aislebot_full.launch.py'"
add_line "alias ab-build='cd ${ROS_WS} && colcon build --symlink-install && source install/setup.bash && cd -'"
add_line "alias ab-log='tail -f ~/aislebot_boot.log'"
add_line "alias ab-status='sudo systemctl status aislebot'"
add_line "alias ab-start='sudo systemctl start aislebot'"
add_line "alias ab-stop='sudo systemctl stop aislebot'"
add_line "alias ab-ports='ls -la /dev/esp32 /dev/mega 2>/dev/null || echo \"no devices found\"'"

ok ".bashrc updated with ROS2 env + aliases"

mkdir -p "$HOME/aislebot_logs"
ok "~/aislebot_logs created"

rm -rf "$CLONE_DIR"

# ════════════════════════════════════════════════════════════════════════
#  DONE
# ════════════════════════════════════════════════════════════════════════
echo ""
echo -e "${G}${B}"
echo "  ╔══════════════════════════════════════════════════════╗"
echo "  ║              INSTALL COMPLETE  ✓                     ║"
echo "  ╚══════════════════════════════════════════════════════╝"
echo -e "${N}"
echo "  Finished : $(date)"
echo "  Log file : ${INSTALL_LOG}"
echo ""
echo "  ── Next steps ───────────────────────────────────────────"
echo ""
echo "  1. Open a NEW terminal  (or: source ~/.bashrc)"
echo ""
echo "  2. Plug in hardware:"
echo "     ESP32 (CP2102) FIRST  →  /dev/esp32"
echo "     Mega  (CH340)  SECOND →  /dev/mega"
echo "     Verify: ab-ports"
echo ""
echo "  3. Flash firmware (from Windows laptop):"
echo "     aislebot_esp32_v2.ino  →  ESP32 @ 921600 baud"
echo "     aislebot_arm_v7.ino    →  Mega  @ 115200 baud"
echo ""
echo "  4. Launch:"
echo "     ab"
echo ""
PI_IP=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "<PI_IP>")
echo "  5. Phone dashboard:"
echo "     http://${PI_IP}:8080"
echo ""
echo "  NOTE: Log out and back in if serial/LCD access fails."
echo "        (dialout/i2c group changes need re-login)"
echo ""