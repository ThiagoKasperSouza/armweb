FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-lc"]

# ros2_control stack + tools
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-controller-manager \
      ros-humble-joint-state-publisher \
      ros-humble-robot-state-publisher \
      ros-humble-xacro \
      ros-humble-tf2-ros \
      ros-humble-tf2-tools \
      ros-humble-foxglove-bridge \
      python3-pip \
      python3-numpy \
      jq \
      curl \
      ca-certificates \
      net-tools \
      iproute2 \
      less \
      vim-tiny \
    && rm -rf /var/lib/apt/lists/*

# OpenUSD (pxr) - CPU only, no GPU needed
RUN python3 -m pip install --no-cache-dir \
      usd-core \
    && python3 -c "from pxr import Usd; print('OpenUSD', Usd.GetVersion())"

# Mesh loading for real robot descriptions (STL collision meshes, DAE visual
# meshes). trimesh reads STL directly; DAE needs the pycollada backend.
RUN python3 -m pip install --no-cache-dir \
      trimesh \
      numpy-stl \
      pycollada \
      lxml \
    && python3 -c "import trimesh, stl, collada; print('trimesh', trimesh.__version__, '| collada', collada.__version__)"

# Real, open-source robot descriptions with production-quality meshes.
# franka_description is Apache-2.0; ur_description ships STL+DAE for UR arms.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-franka-description \
      ros-humble-ur-description \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/\$ROS_DISTRO/setup.bash" >> /root/.bashrc

WORKDIR /ws
RUN mkdir -p /ws/src /ws/logs /ws/data