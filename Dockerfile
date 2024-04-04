ARG ROS_DISTRIBUTION=humble

FROM ros:$ROS_DISTRIBUTION-ros-base

# Install clang and set as default compiler.
RUN apt-get update && apt-get install -y --no-install-recommends \
  clang \
  && rm -rf /var/lib/apt/lists/*

ENV CC=clang
ENV CXX=clang++

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  && rm -rf /var/lib/apt/lists/*

# Install other packages


# Set environment and working directory
ENV ROS_WS /ros2_ws
ENV ROS_WS_SRC /ros2_ws/src
WORKDIR $ROS_WS_SRC

# Only to use not cached git clone
ADD "https://api.github.com/repos/Hercogs/diffdrive_roboteq_sbl/commits?per_page=1" latest_commit
# Add common files and source code
RUN git clone https://github.com/Hercogs/diffdrive_roboteq_sbl.git

WORKDIR $ROS_WS

# Install rosdep dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep update --include-eol-distros && rosdep install -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Build workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# source workspace from entrypoint
RUN sed --in-place \
      's|^source .*|source "$ROS_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

# ENTRYPOINT ["sh", "-c", "./install/setup.sh"]
CMD ["ros2", "launch", "diffdrive_roboteq_sbl", "diffbot.launch.py"]



# TODO write docker compose


# docker build -f Dockerfile -t diffdrive_roboteq_sbl .

# xhost +local:root

# docker run --rm -it --privileged -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY diffdrive_roboteq_sbl bash