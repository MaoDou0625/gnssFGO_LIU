FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# install dep
RUN apt update \
    && apt install -y \
    libgtk2.0-dev \
    freeglut3-dev \
    mesa-utils \
    ros-humble-ublox-serialization \
    libgeographic-dev \
    ros-humble-pcl-msgs \
    ros-humble-pcl-conversions \
    ros-humble-gps-msgs \
    ros-humble-nmea-msgs \
    ros-humble-novatel-gps-driver \
    ros-humble-novatel-gps-msgs \
    ros-humble-camera-calibration-parsers \
    libgoogle-glog-dev \
    ros-humble-geographic-msgs \
    ros-humble-marti-nav-msgs \
    libgeos++-dev  \
    libgeos3.10.2  \
    libgeos-c1v5  \
    libgeos-dev  \
    libgeos-doc \
    xterm \
    ninja-build \
    gdb

# eigen setup
RUN cd /usr/local/include \ 
    && ln -sf /usr/include/eigen3/Eigen Eigen \
    && ln -sf /usr/include/eigen3/unsupported unsupported

# build adapted mapviz
RUN mkdir -p /workspace/mapviz_ws/src \
    && cd /workspace/mapviz_ws/src \
    && git clone -b humble-dev https://github.com/rwth-irt/mapviz.git \
    && git clone https://github.com/swri-robotics/marti_common.git \
    && git clone https://github.com/swri-robotics/marti_messages.git \
    && cd /workspace/mapviz_ws \
    && source /opt/ros/humble/setup.bash \
    && colcon build

# build adapted gtsam
RUN mkdir -p /workspace/thirdparty \
    && cd /workspace/thirdparty \
    && git clone https://github.com/rwth-irt/gtsam.git \
    && cd /workspace/thirdparty/gtsam \
    && cmake -B build -DGTSAM_USE_SYSTEM_EIGEN=ON \
    && cmake --build build -j \
    && cmake -E chdir "build" make install -j

# build NumCpp
RUN cd /workspace/thirdparty \ 
    && git clone https://github.com/dpilger26/NumCpp.git \
    && cd /workspace/thirdparty/NumCpp \
    && cmake -B build \ 
    && cmake --build build --target install 

# final update gnssFGO
RUN echo "source /workspace/mapviz_ws/install/setup.bash" >> ~/.bashrc \
    && echo "source /workspace/fgo_ws/install/setup.bash" >> ~/.bashrc

# setup CMD for new shell
CMD ["/usr/bin/bash"]


