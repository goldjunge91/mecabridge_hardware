#!/usr/bin/env bash
# ROS 2 installer for Raspberry Pi 4B on Ubuntu Server (24.04/22.04)
# - Installs ROS 2 (Jazzy on 24.04 / Humble on 22.04), dev tools, useful drivers
# - Enables I2C & SPI, sets up pigpio daemon, camera utilities
# - Sets CycloneDDS as default RMW
#
# Usage:
#   bash install_ros2_pi.sh                 # minimal (ros-base + test tools)
#   bash install_ros2_pi.sh desktop         # full desktop variant (heavier)
#
# Context & verify steps for this project: i2c scan, pigpio PWM, ros2 image_tools cam2image. :contentReference[oaicite:0]{index=0}
# Project background (Pi 4B + ROS 2 stack used for the robot): :contentReference[oaicite:1]{index=1} :contentReference[oaicite:2]{index=2} :contentReference[oaicite:3]{index=3}

set -Eeuo pipefail
trap 'echo "[ERROR] Failed at line $LINENO" >&2' ERR

# Re-run with sudo if needed
if [[ ${EUID} -ne 0 ]]; then
  exec sudo -E bash "$0" "${@}"
fi

# ----- Detect Ubuntu release → choose ROS distro -----
. /etc/os-release
UBU_CODENAME="${UBUNTU_CODENAME:-}"
case "$UBU_CODENAME" in
  noble)  ROS_DISTRO="jazzy"  ;;  # Ubuntu 24.04
  jammy)  ROS_DISTRO="humble" ;;  # Ubuntu 22.04
  *) echo "[FATAL] Unsupported Ubuntu codename '$UBU_CODENAME'. Use 22.04 (jammy) or 24.04 (noble)." >&2; exit 1 ;;
esac

# Variant selection
VARIANT="${1:-base}"
case "$VARIANT" in
  base)    ROS_VARIANT_PKG="ros-${ROS_DISTRO}-ros-base" ;;
  desktop) ROS_VARIANT_PKG="ros-${ROS_DISTRO}-desktop"  ;;
  *) echo "[FATAL] Unknown argument '$VARIANT' (use 'base' or 'desktop')." >&2; exit 1 ;;
esac

export DEBIAN_FRONTEND=noninteractive

echo "[1/8] Updating APT and base packages…"
apt-get update -y
apt-get install -y --no-install-recommends \
  ca-certificates gnupg2 curl wget lsb-release software-properties-common locales

# ----- Locale (UTF-8) -----
if ! locale -a | grep -q 'en_US.utf8'; then
  echo "[2/8] Generating locale en_US.UTF-8…"
  locale-gen en_US.UTF-8
  update-locale LANG=en_US.UTF-8
fi

# ----- Add ROS 2 apt repository -----
echo "[3/8] Adding ROS 2 apt repository for $UBU_CODENAME ($ROS_DISTRO)…"
install -m 0755 -d /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBU_CODENAME} main" \
  > /etc/apt/sources.list.d/ros2.list
apt-get update -y

# ----- Install ROS 2 + dev tools + useful extras for Pi robotics -----
echo "[4/8] Installing ROS 2 ($ROS_VARIANT_PKG) and tools…"
apt-get install -y --no-install-recommends \
  $ROS_VARIANT_PKG \
  ros-${ROS_DISTRO}-image-tools \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-demo-nodes-cpp \
  ros-${ROS_DISTRO}-rqt-image-view \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  python3-rosdep python3-colcon-common-extensions python3-vcstool \
  build-essential cmake git  v4l-utils ffmpeg gstreamer1.0-tools \
  python3-opencv

# Ensure 'universe' is enabled so packages like 'pigpio' are available
add-apt-repository -y universe || true
apt-get update -y
# Force building pigpio from source (APT package unreliable on some images)
echo "[4.1/8] Installing pigpio tools and building pigpio from source (forced)..."
apt-get update -y
apt-get install -y --no-install-recommends \
  pigpio-tools python3-pigpio libpigpiod-if2-1 libpigpiod-if-dev \
  python3-setuptools build-essential git curl|| true

TMPDIR="$(mktemp -d)"
cleanup() { rm -rf "$TMPDIR"; }
trap cleanup EXIT

git clone --depth=1 https://github.com/joan2937/pigpio "$TMPDIR/pigpio"
make -C "$TMPDIR/pigpio" -j"$(nproc)"
make -C "$TMPDIR/pigpio" install
ldconfig || true

# Ensure pigpiod is reachable at a stable path
if [ -x "$(command -v pigpiod)" ] && [ "$(command -v pigpiod)" != "/usr/bin/pigpiod" ]; then
  ln -sf "$(command -v pigpiod)" /usr/bin/pigpiod
fi

# # Install systemd unit if missing
# if [ ! -f /etc/systemd/system/pigpiod.service ]; then
#   cat >/etc/systemd/system/pigpiod.service <<'UNIT'
# [Unit]
# Description=Daemon required to control GPIO pins via pigpio
# After=network.target

# [Service]
# Type=forking
# ExecStart=/usr/bin/pigpiod -l
# #ExecStart=${PG_BIN} -l -n 127.0.0.1
# PIDFile=/run/pigpio.pid
# Restart=on-failure

# [Install]
# WantedBy=multi-user.target
# UNIT
# fi
# Install systemd unit if missing
if [ ! -f /etc/systemd/system/pigpiod.service ]; then
  PG_BIN="$(command -v pigpiod || echo /usr/local/bin/pigpiod)"
  cat >/etc/systemd/system/pigpiod.service <<UNIT
[Unit]
Description=Daemon required to control GPIO pins via pigpio
After=network.target

[Service]
Type=forking
# ensure we remove stale pid before start and use resolved pigpiod path
ExecStartPre=/bin/rm -f /run/pigpio.pid /var/run/pigpio.pid
ExecStart=${PG_BIN} -l -n 127.0.0.1
PIDFile=/run/pigpio.pid
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT
fi

systemctl daemon-reload
systemctl enable --now pigpiod || true

# Install remaining sensor/camera utils
apt-get install -y --no-install-recommends i2c-tools python3-smbus v4l-utils || true

# ----- rosdep init/update -----
echo "[5/8] Initializing rosdep…"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
if [[ -n "${SUDO_USER-}" && "$SUDO_USER" != "root" ]]; then
  su - "$SUDO_USER" -c "rosdep update"
else
  rosdep update
fi

# ----- Set environment defaults (source + CycloneDDS) -----
echo "[6/8] Creating /etc/profile.d/ros2.sh…"
cat >/etc/profile.d/ros2.sh <<EOF
# Auto-sourced ROS 2 environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  . /opt/ros/${ROS_DISTRO}/setup.bash
fi
# Use CycloneDDS by default (good on Raspberry Pi)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
EOF
chmod 0644 /etc/profile.d/ros2.sh

# # ----- Enable I2C/SPI on Ubuntu for Raspberry Pi -----
# echo "[7/8] Enabling I2C & SPI overlays…"
# FWCFG="/boot/firmware/usercfg.txt"
# touch "$FWCFG"
# grep -q '^dtparam=i2c_arm=on' "$FWCFG" || echo 'dtparam=i2c_arm=on' >> "$FWCFG"
# grep -q '^dtparam=spi=on'     "$FWCFG" || echo 'dtparam=spi=on'     >> "$FWCFG"

# Add current user to helpful groups (if they exist)
TARGET_USER="${SUDO_USER:-root}"
for grp in i2c dialout video gpio; do
  if getent group "$grp" >/dev/null 2>&1 && id -nG "$TARGET_USER" | tr ' ' '\n' | grep -qx "$grp"; then
    : # already in group
  elif getent group "$grp" >/dev/null 2>&1; then
    usermod -aG "$grp" "$TARGET_USER" || true
  fi
done

# Enable pigpio daemon now and on boot
systemctl enable pigpiod >/dev/null 2>&1 || true
systemctl restart pigpiod >/dev/null 2>&1 || true

# ----- Done -----
echo
echo "✅ ROS 2 ($ROS_DISTRO, $VARIANT) install complete on Ubuntu $UBU_CODENAME."
echo "   Environment will auto-load next login via /etc/profile.d/ros2.sh"
echo
echo "ℹ Recommended next steps (from project verify notes):"  # :contentReference[oaicite:4]{index=4}
echo "   1) Reboot once to apply I2C/SPI overlays."
echo "   2) After login:  ros2 --version"
echo "   3) USB cam test: ros2 run image_tools cam2image --ros-args -p frequency:=5"
echo "   4) I2C scan:     sudo i2cdetect -y 1"
echo "   5) pigpio test:  pigs pfs 18 200 && pigs p 18 64 && pigs p 18 0"
echo
echo "Reboot now? [Y/n]"
read -r ans
if [[ -z "${ans}" || "${ans,,}" == "y" || "${ans,,}" == "yes" ]]; then
  systemctl reboot
fi
