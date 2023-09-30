FROM ghcr.io/vortexntnu/vortex-docker-images:ros2

# Create workspace
COPY . /ros2_ws/src/vortex-m3-sonar-driver

# Install rosdeps
WORKDIR /ros2_ws
RUN ls
RUN ros2
RUN cat ~/.bashrc
RUN sudo rosdep init && rosdep update && rosdep install --from-paths src -y --ignore-src