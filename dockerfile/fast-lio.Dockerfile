FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

ARG BUILD_FAST_LIO=0

RUN apt-get -o Acquire::Retries=5 update && apt-get install -y --no-install-recommends --fix-missing \
    build-essential \
    cmake \
    git \
    nano \
    pkg-config \
    python3-catkin-tools \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    ros-noetic-rosbash \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/catkin_ws
COPY . /root/catkin_ws

RUN chmod +x /root/catkin_ws/run_build.sh /root/catkin_ws/scripts/*.sh

RUN if [ "${BUILD_FAST_LIO}" = "1" ]; then \
      /root/catkin_ws/scripts/bootstrap_fastlio_deps.sh; \
    fi

RUN source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/catkin_ws
CMD ["bash"]
