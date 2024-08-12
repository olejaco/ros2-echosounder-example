FROM ros:humble-ros-base

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app

# Copy the package files
COPY src/echo_sounder_pkg /app/src/echo_sounder_pkg

# Install the package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /app/src/echo_sounder_pkg && \
    colcon build && \
    source install/setup.bash"

# Create a new entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /app/src/echo_sounder_pkg/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# The default command (can be overridden)
CMD ["ros2", "run", "echo_sounder_pkg", "sensor_node"]