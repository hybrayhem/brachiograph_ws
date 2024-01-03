# FROM arm64v8/ros:noetic-ros-base-focal
FROM ros:noetic-ros-base-focal 

# Set default shell
SHELL ["/bin/bash", "-c"]

# Install ros tools
RUN apt update && apt install -y \
    git \
    ros-noetic-rviz \
    ros-noetic-xacro \
    ros-noetic-catkin \
    && rm -rf /var/lib/apt/lists/*

# Install gazebo
RUN apt update && apt install -y \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Install project dependencies
RUN apt update && apt install -y \
    ros-noetic-controller-manager \
    ros-noetic-position-controllers \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-controller \
    ros-noetic-robot-state-publisher \
    ros-noetic-robot-state-controller \
    ros-noetic-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# Prepare rosdep
RUN rosdep update

# Source ros
# RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc # Already done in ros:noetic-ros-base-focal

# Clone and setup the catkin workspace
RUN cd /root && git clone https://github.com/hybrayhem/brachiograph_ws.git
WORKDIR /root/brachiograph_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/noetic/setup.bash && catkin_make
RUN echo "source /root/brachiograph_ws/devel/setup.bash" >> /root/.bashrc

# CMD ["/bin/bash"]
# CMD [ "/app/start.sh" ]
COPY start.sh /app/start.sh
ENTRYPOINT [ "/app/start.sh" ]