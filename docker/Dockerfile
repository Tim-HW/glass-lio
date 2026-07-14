# glasslio dev image.
#
# Pinned to ROS 2 Jazzy on Ubuntu Noble, which is what the project is developed and
# tested against. PCL 1.14 and Eigen 3.4 come from that distro -- pinning the base
# image is what makes "works on my machine" reproducible for everyone else.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Runtime + build deps.
#
#   rviz2 / rosbag2      -- ros-base does not include either, and run_bag.sh needs both
#   sqlite3 plugin       -- the test bag is a .db3 (rosbag2's default sqlite storage)
#   curl unzip           -- download_bag.sh
#   uncrustify           -- the lint the test suite actually runs
#
# The package-specific deps (pcl_conversions, tf2_eigen, ...) are installed below by
# rosdep, straight from package.xml -- so this list never drifts from the manifest.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      curl \
      wget \
      unzip \
      sudo \
      gdb \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-jazzy-rviz2 \
      ros-jazzy-rosbag2 \
      ros-jazzy-rosbag2-storage-default-plugins \
      ros-jazzy-ament-uncrustify \
      ros-jazzy-ament-lint-auto \
      ros-jazzy-ament-lint-common \
    && rm -rf /var/lib/apt/lists/*

# Resolve the package's own dependencies FROM package.xml, so this image cannot drift
# out of sync with the manifest. Copying only the manifest keeps this layer cached
# across source edits.
WORKDIR /ws
COPY package.xml /tmp/glasslio/package.xml
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /tmp/glasslio --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/* /tmp/glasslio

# The `ubuntu` user (uid 1000) ships with the Noble base image and normally matches the
# host user, so files created in the bind-mounted workspace are not left root-owned.
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && mkdir -p /ws && chown -R ubuntu:ubuntu /ws

USER ubuntu

# Auto-source ROS and, once it exists, this workspace's overlay -- so `ros2 run glasslio`
# and the `lio` shared library resolve without the user having to think about it.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> ~/.bashrc \
    && echo 'cd /ws' >> ~/.bashrc

WORKDIR /ws
CMD ["/bin/bash"]
