# FROM arm64v8/ros:noetic-ros-base-focal
FROM ros:noetic-ros-base-focal 

# Set default shell
SHELL ["/bin/bash", "-c"]

# ----------------------------- Install ros tools ---------------------------- #
RUN apt update && apt install -y \
    git \
    ros-noetic-rviz \
    ros-noetic-xacro \
    ros-noetic-catkin \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------ Install gazebo ------------------------------ #
RUN apt update && apt install -y \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# ----------------------- Install project dependencies ----------------------- #
RUN apt update && apt install -y \
    ros-noetic-controller-manager \
    ros-noetic-position-controllers \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-controller \
    ros-noetic-robot-state-publisher \
    ros-noetic-robot-state-controller \
    ros-noetic-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------- #
#                     Clone and Setup The Catkin Workspace                     #
# ---------------------------------------------------------------------------- #
# Prepare rosdep
RUN rosdep update

# Source ros
# RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc # Already done in ros:noetic-ros-base-focal

# Clone or copy the workspace
RUN cd /root && git clone https://github.com/hybrayhem/brachiograph_ws.git && echo "abc"

# Build the workspace
WORKDIR /root/brachiograph_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/noetic/setup.bash && catkin_make
RUN echo "source /root/brachiograph_ws/devel/setup.bash" >> /root/.bashrc


# -------------------------------- Launch -------------------------------- #
COPY start.sh /app/start.sh
ENTRYPOINT [ "/app/start.sh" ]
# or CMD [ "/app/start.sh" ]