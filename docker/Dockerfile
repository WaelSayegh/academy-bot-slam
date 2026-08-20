# =============================================================================
# Robotics Academy — Summer 2026 — SLAM Track
# ROS 2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox development image
# =============================================================================
#
# Base: official ROS 2 Jazzy "ros-base" image (Ubuntu 24.04 Noble). It is a
# lean core; user_install.sh layers on exactly what this project needs — rviz2,
# Gazebo Harmonic (ros-gz), Nav2 and slam_toolbox — which keeps the image and
# its download smaller than the all-in-one desktop-full base.
#
# Build with:   ./build_ros_image.sh
# Run with:     ./run_ros_container.sh
# =============================================================================
FROM ros:jazzy-ros-base
# it is lightweight since it is headless

LABEL org.opencontainers.image.title="acadbot-jazzy"
LABEL org.opencontainers.image.description="ROS 2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox for the Robotics Academy SLAM track"
LABEL maintainer="wael.alsayegh@inmind.ai"

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

# A non-root developer user with passwordless sudo. UID/GID 1000 matches the
# typical first Linux user, which keeps mounted-workspace file ownership sane.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# -----------------------------------------------------------------------------
# 1. Install all system / ROS dependencies (see user_install.sh)
# -----------------------------------------------------------------------------
COPY user_install.sh /tmp/user_install.sh
RUN chmod +x /tmp/user_install.sh && /tmp/user_install.sh && rm /tmp/user_install.sh

# -----------------------------------------------------------------------------
# 2. Create the developer user
#    Ubuntu 24.04 (Noble) base images ship a default 'ubuntu' user at UID 1000,
#    which collides with our target UID — so drop any existing user that owns
#    USER_UID first, then (re)create ours cleanly.
# -----------------------------------------------------------------------------
RUN set -eux; \
    if getent passwd ${USER_UID} >/dev/null; then \
        userdel -r "$(getent passwd ${USER_UID} | cut -d: -f1)" 2>/dev/null || true; \
    fi; \
    if ! getent group ${USER_GID} >/dev/null; then \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    fi; \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}; \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# -----------------------------------------------------------------------------
# 3. Shell environment: source ROS + the project workspace automatically
# -----------------------------------------------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash"               >> /home/${USERNAME}/.bashrc && \
    echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export GZ_VERSION=harmonic"                              >> /home/${USERNAME}/.bashrc && \
    echo "export ROS_DOMAIN_ID=42"                                 >> /home/${USERNAME}/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export PS1='\[\033[01;32m\]acadbot\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /home/${USERNAME}/.bashrc

# -----------------------------------------------------------------------------
# 4. Workspace mount point (the real source is bind-mounted at run time)
# -----------------------------------------------------------------------------
RUN mkdir -p /ros2_ws/src && chown -R ${USER_UID}:${USER_GID} /ros2_ws

USER ${USERNAME}
WORKDIR /ros2_ws

# A login shell so .bashrc (ROS sourcing) is picked up.
CMD ["/bin/bash"]
