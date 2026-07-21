#FROM ros:humble-ros-base AS builder
FROM osrf/ros:humble-desktop-full AS builder


ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    FRANKA_ROS2_VER=v2.1.0 \
    LIBFRANKA_VER=0.20.5 \
    FRI_CLIENT_VERSION=1.15

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-argcomplete \
        git \
        curl \
        sudo \
        build-essential \
        cmake \
        ros-dev-tools \
        mesa-utils 
        # && rm -rf /var/lib/apt/lists/*

RUN apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-sdformat-urdf \
        ros-humble-joint-state-publisher-gui \
        ros-humble-ros2controlcli \
        ros-humble-controller-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-controller-manager \
        ros-humble-ros2-control-test-assets \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
        ros-humble-hardware-interface \
        ros-humble-control-msgs \
        ros-humble-backward-ros \
        ros-humble-generate-parameter-library \
        ros-humble-realtime-tools \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-joint-trajectory-controller \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-ur-description

# Install libfranka
RUN curl -L https://github.com/frankarobotics/libfranka/releases/download/${LIBFRANKA_VER}/libfranka_${LIBFRANKA_VER}_$(lsb_release -cs)_amd64.deb -o libfranka.deb \
    && apt-get install ./libfranka.deb -y \
    && rm libfranka.deb

# --------------------------
# franka_ws
# --------------------------
WORKDIR /franka_ws
RUN git clone --branch ${FRANKA_ROS2_VER} --recursive https://github.com/frankarobotics/franka_ros2 src
RUN mkdir src/libfranka && touch src/libfranka/COLCON_IGNORE


RUN if dpkg --compare-versions ${FRANKA_ROS2_VER#v} ge 2.2.0; then \
      vcs import src < src/dependency.repos --recursive --skip-existing; \
    else \
      vcs import src < src/franka.repos --recursive --skip-existing; \
    fi
RUN rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro ${ROS_DISTRO} \
    --dependency-types build \
    --dependency-types buildtool \
    -y

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install \
    --packages-skip franka_example_controllers franka_ros2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

# --------------------------
# lbr_ws
# --------------------------
WORKDIR /lbr_ws
RUN mkdir -p src && \
    vcs import src --input https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml

RUN rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro ${ROS_DISTRO} \
    -y

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

# --------------------------
# controllers_ws
# --------------------------
WORKDIR /controllers_ws
COPY . /controllers_ws/src/compliant_controllers

RUN vcs import src --input src/compliant_controllers/repos-compliant-controllers.yaml --skip-existing
# When public change to:
#vcs import src --input https://raw.githubusercontent.com/smihael/compliant_controllers/master/repos-compliant-controllers.yaml 

RUN rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro ${ROS_DISTRO} \
    --dependency-types build \
    --dependency-types buildtool \
    --dependency-types exec \
    -y

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /franka_ws/install/setup.bash && \
    source /lbr_ws/install/setup.bash && \
    colcon build --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"


# --------------------------
# Export runtime dependencies
# --------------------------
# This extracts ONLY exec dependencies as apt packages
#RUN rosdep install \
#    --from-paths /franka_ws/src /lbr_ws/src /controllers_ws/src \
#    --ignore-src \
#    --rosdistro ${ROS_DISTRO} \
#    --dependency-types exec \
#    --simulate > /tmp/runtime_deps.sh


# ==========================
# Stage 2: Runtime (minimal)
# ==========================
# This stage is meant to be as minimal as possible, only containing runtime dependencies and the merged install spaces.
# But it makes no sense until https://github.com/gazebosim/ros_gz/issues/874 is not resolved

# FROM osrf/ros:humble-desktop-full

# ENV DEBIAN_FRONTEND=noninteractive \
#     ROS_DISTRO=humble

# # Install ONLY runtime dependencies (no rosdep needed)
# COPY --from=builder /tmp/runtime_deps.sh /tmp/runtime_deps.sh

# RUN apt-get update && \
#     /bin/bash /tmp/runtime_deps.sh && \
#     apt-get install -y --no-install-recommends \
#         ros-humble-ros-gz-sim \
#         ros-humble-xacro &&\
#     apt-get clean && \
#     rm -rf /var/lib/apt/lists/*

# # Copy merged install spaces only
# COPY --from=builder /franka_ws/install /franka_ws/install
# COPY --from=builder /lbr_ws/install /lbr_ws/install
# COPY --from=builder /controllers_ws/install /controllers_ws/install


RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /franka_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /lbr_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /controllers_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc && \
    chown -R ${USER_UID}:${USER_GID} /home/${USERNAME} /controllers_ws

WORKDIR /controllers_ws

SHELL ["/bin/bash", "-c"]
