FROM osrf/ros:jazzy-desktop-full-noble

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-dev-tools \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-tf2-tools \
    libgl1-mesa-dri libglx-mesa0 \
    python3-colcon-common-extensions python3-rosdep python3-pip git-lfs \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update || true

RUN if id -u $USERNAME >/dev/null 2>&1; then \
        true; \
    elif id -u ubuntu >/dev/null 2>&1; then \
        usermod -l $USERNAME ubuntu && \
        groupmod -n $USERNAME ubuntu && \
        usermod -d /home/$USERNAME -m $USERNAME && \
        echo "Renamed ubuntu -> $USERNAME"; \
    else \
        useradd --uid $USER_UID -m -s /bin/bash $USERNAME; \
    fi \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME

WORKDIR /home/$USERNAME/spider_bot_bringup

COPY --chown=$USER_UID . ./src

RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends ros-jazzy-plotjuggler \
    && sudo rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages --ignore-installed open3d

RUN sudo rosdep fix-permissions; rosdep update || echo "rosdep update skipped (rate-limited)"

RUN sudo apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y || echo "rosdep install had issues (some deps may be missing)" \
    && sudo rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source /home/$USERNAME/spider_bot_bringup/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]