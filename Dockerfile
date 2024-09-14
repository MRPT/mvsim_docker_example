# Dockerfile for ROS 2 Humble + mvsim
FROM ros:humble

# Install necessary dependencies for mvsim
RUN apt-get update && apt-get install -y \
    ros-humble-mvsim \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create working directories
WORKDIR /ros2_ws/src

# Copy the launch.py and the Python script to the image
COPY launch_mvsim.launch.py /ros2_ws/src/
COPY robot_commander.py /ros2_ws/src/

# Return to the workspace root directory
WORKDIR /ros2_ws

# Build the workspace
RUN colcon build

# Source ROS 2 and setup the environment for the launch file
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command to launch the ROS 2 simulation
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch src/launch_mvsim.launch.py"]