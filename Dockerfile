ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO} AS deps

# Set working directory
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]

# Copy only the src folder
COPY src/ /root/ros2_ws/src/

# Copy the Panda_arm.zip file into the container
COPY Panda_arm.zip /tmp/

# Copy local yolo_models folder contents into container
COPY yolo_models/ /root/yolo_models/

# Install dependencies
RUN apt-get update
RUN apt-get -y --quiet --no-install-recommends install python3 python3-pip
RUN rosdep install --from-paths src --ignore-src -r -y

RUN if [ "$(lsb_release -rs)" = "24.04" ] || [ "$(lsb_release -rs)" = "24.10" ]; then \
    pip3 install -r src/requirements.txt --break-system-packages --ignore-installed; \
    else \
    pip3 install -r src/requirements.txt; \
    fi

# Build the ws with colcon
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# Create folder for Gazebo model and unzip
RUN mkdir -p ~/gazebo_models/panda_arm && \
    unzip /tmp/Panda_arm.zip -d ~/gazebo_models/panda_arm && \
    rm /tmp/Panda_arm.zip

# Source the ROS 2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["bash"]
