#!/usr/bin/env bash
#
# user_install.sh
# -----------------------------------------------------------------------------
# Installs every system dependency the Robotics Academy SLAM project needs on top
# of a ROS 2 Jazzy base image:
#   * Gazebo Harmonic (gz-sim 8) + ros_gz integration  -> simulation
#   * Nav2 (navigation2 + nav2_bringup)                 -> navigation & recovery
#   * slam_toolbox                                      -> LiDAR SLAM
#   * robot_state_publisher / xacro / teleop / twist_mux-> robot plumbing
#   * build & dev tooling (colcon, rosdep, clang, gdb)  -> C++ development
#
# It is written to be run *inside* the Dockerfile, but it is also safe to run by
# hand on a fresh Ubuntu 24.04 (Noble) + ROS 2 Jazzy machine:
#
#     sudo ./user_install.sh
#
# Re-running is harmless (apt is idempotent).
# -----------------------------------------------------------------------------
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
GZ_VERSION="harmonic"

echo "==> Installing dependencies for ROS 2 ${ROS_DISTRO} + Gazebo ${GZ_VERSION}"

export DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------------------------
# 0. Make apt resilient to flaky mirrors / dropped connections.
#    packages.ros.org occasionally drops connections mid-fetch; retries + a
#    download loop turn a transient network blip into a slower (not failed) build.
# -----------------------------------------------------------------------------
cat > /etc/apt/apt.conf.d/80-acadbot-retries <<'EOF'
Acquire::Retries "12";
Acquire::http::Timeout "90";
Acquire::https::Timeout "90";
Acquire::ftp::Timeout "90";
EOF

# apt_install <packages...> — retry the whole group a few times, refreshing the
# package lists with --fix-missing between attempts.
apt_install() {
    local attempt
    for attempt in 1 2 3 4 5 6; do
        if apt-get install -y --no-install-recommends "$@"; then
            return 0
        fi
        echo "==> apt install attempt ${attempt} failed; refreshing and retrying..."
        apt-get update --fix-missing || true
        sleep 8
    done
    echo "==> apt install still failing after retries" >&2
    return 1
}

# -----------------------------------------------------------------------------
# 1. Base utilities + apt over HTTPS
# -----------------------------------------------------------------------------
apt-get update
apt_install \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    ca-certificates \
    software-properties-common \
    sudo \
    nano \
    vim \
    git \
    tmux \
    htop \
    iproute2 \
    python3-pip

# -----------------------------------------------------------------------------
# 2. Gazebo Harmonic apt repository (OSRF)
#    ros-jazzy-ros-gz is built against Gazebo Harmonic, whose libraries live in
#    the OSRF packages repo, so we register it explicitly.
# -----------------------------------------------------------------------------
echo "==> Adding the OSRF Gazebo apt repository"
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

apt-get update

# -----------------------------------------------------------------------------
# 3. ROS 2 development tooling
# -----------------------------------------------------------------------------
echo "==> Installing build & development tooling"
apt_install \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    build-essential \
    cmake \
    clang \
    clangd \
    clang-format \
    gdb

# -----------------------------------------------------------------------------
# 4. The ROS 2 packages the project depends on
# -----------------------------------------------------------------------------
echo "==> Installing ROS 2 ${ROS_DISTRO} packages (sim, nav, slam, tooling)"
apt_install \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-image \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-rviz2

# builtin robot frame publisher, takes all links joints... and publishes them on a topic
# any package that has a urdf reads from this topic
# xacro and a urdf are very important for robot describtion
# they are a framework that are called a universal robotic describtion 
# to describe a robot in a lang that ros2 can understand that is xacro, 
# macros are functions that can be called in a urdf file to generate a robot model


# -----------------------------------------------------------------------------
# 5. rosdep (so students can `rosdep install` their own future packages)
# -----------------------------------------------------------------------------
echo "==> Initialising rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init || true
fi

# -----------------------------------------------------------------------------
# 6. Clean up apt caches to keep the image small
# -----------------------------------------------------------------------------
echo "==> Cleaning apt caches"
apt-get clean
rm -rf /var/lib/apt/lists/*

echo "==> user_install.sh finished successfully"
