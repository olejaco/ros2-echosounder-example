FROM osrf/ros:humble-desktop

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    bash-completion \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up the development environment
USER $USERNAME

# Set up ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the working directory
WORKDIR /workspace