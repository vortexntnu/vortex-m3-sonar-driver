FROM ghcr.io/vortexntnu/vortex-docker-images:ros2 

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip

# Create workspace
COPY . /ros2_ws/src/vortex-m3-sonar-driver

# Install rosdeps
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src -y --ignore-src

