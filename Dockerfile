ARG ROS_DISTRIBUTION="jazzy"

FROM ghcr.io/sloretz/ros:${ROS_DISTRIBUTION}-desktop-full-2026-02-01 AS devcontainer

RUN mkdir -p /ros2_ws/src

ARG ROS_DISTRIBUTION
ARG FRANKA_ROS2_VERSION="v3.4.0"

# Add non-root user
ARG USERNAME=franka
ARG USER_UID=1001
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

WORKDIR "/home/${USERNAME}/ros2_ws/src"
RUN /bin/bash -c '\
    git clone https://github.com/frankarobotics/franka_ros2.git --branch ${FRANKA_ROS2_VERSION} && \
    (cd franka_ros2 && git checkout ${FRANKA_ROS2_VERSION} && git submodule update) && \
    vcs import < franka_ros2/dependency.repos && \
    find . -name ".git" -type d -execdir git submodule update --init --recursive \; && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/'

WORKDIR "/home/${USERNAME}/ros2_ws"

RUN apt-get update && apt-get install -y wget ros-${ROS_DISTRIBUTION}-rmw-cyclonedds-cpp && rosdep install --from-paths src --ignore-src -r -y && apt-get clean && rm -rf /var/lib/apt/lists/*

USER ${USERNAME}

RUN /bin/bash -c '\
    echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.sh" >> /home/${USERNAME}/.profile && \
    echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /home/${USERNAME}/ros2_ws/install/setup.sh" >> /home/${USERNAME}/.profile'

SHELL ["/bin/bash", "-l", "-c"]

FROM devcontainer AS application

COPY . src/franka_follower_controllers/

RUN source /opt/ros/${ROS_DISTRIBUTION}/setup.bash && colcon build --packages-up-to franka_follower_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF

STOPSIGNAL SIGINT

ENTRYPOINT ["/bin/bash", "-l", "-c", "exec ros2 launch franka_follower_controllers follower.launch.py config_file:=/config.yml"]