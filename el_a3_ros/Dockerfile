FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# ROS2 Control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-controller-interface \
    # MoveIt2
    ros-humble-moveit \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-kinematics \
    # Tools
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    # Pinocchio, TF2, Joy
    ros-humble-pinocchio \
    ros-humble-joy \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    # System
    can-utils \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    pyyaml \
    scipy \
    "flask>=2.0" \
    "flask-socketio>=5.0" \
    "python-socketio>=5.0" \
    "eventlet>=0.33"

WORKDIR /ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
