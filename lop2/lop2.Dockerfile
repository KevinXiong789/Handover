
# FROM osrf/ros:humble-desktop-full
# # Differences: desktop vs. desktop-full:
# # https://www.ros.org/reps/rep-2001.html#humble-hawksbill-may-2022-ongoing


# # Use bash shell as default instead of sh shell, for convenience.
# SHELL ["/bin/bash", "-c"]

# # Only during build:
# ARG DEBIAN_FRONTEND=noninteractive



# # Get dependencies:

# RUN apt update && \
#     apt upgrade -y && \
#     apt install -y --no-install-recommends \
#         python3-dev \
#         python3-pip \
#         python3-setuptools \
#         git \
#         g++ \
#         wget \
#         make \
#         nano \
#         cmake \
#         build-essential

# # For Python API:
# RUN python3 -m pip install --upgrade pip



# # Install librealsense:

# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
#     apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# # 'software-properties-common' for 'add-apt-repository':
# RUN apt install -y software-properties-common && \
#     add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
#     apt install -y \
#         librealsense2-dkms \
#         librealsense2-utils \
#         librealsense2-dev



# # ROS setup:

# ARG WS_NAME=ros2_ws


# # Copy the LOP2 package:
# COPY ./lop2/ /${WS_NAME}/src/lop2
# COPY ./lop2_interfaces/ /${WS_NAME}/src/lop2_interfaces
# COPY ../pointcloud_processing/ /${WS_NAME}/src/pointcloud_processing

# # ROS-RealSense-wrapper install:
# WORKDIR /${WS_NAME}/src
# # Release 4.51.1 from 13.09.2022:
# RUN git clone https://github.com/IntelRealSense/realsense-ros.git \
#         --branch 4.51.1 \
#         --depth 1

# # - rosdep init throws an error if it is already initialised, this is ignored.
# # - Setuptools version 58.2.0 is the last version that works with ROS 2 python packages without any warnings
# # because it is the last version that supports the old installation method, "python setup.py install."
# # This method has been deprecated and replaced by newer, standards-based tools, such as pip and ament.
# WORKDIR /${WS_NAME}
# RUN apt install python3-rosdep -y && \
#     (rosdep init || true) && \
#     rosdep update --include-eol-distros && \
#     python3 -m pip install setuptools==58.2.0 \
#     source /opt/ros/humble/setup.bash && \
#     rosdep install -i -y \
#         --from-path src \
#         --rosdistro humble \
#         --skip-keys="librealsense2 lop2_interfaces" && \
#     colcon build



# # Add setup file to shell:
# RUN (echo; echo 'source /opt/ros/humble/setup.bash') >> ~/.bashrc

# # Clear cache:
# RUN rm -rf /var/lib/apt/lists/* /tmp/*


# WORKDIR /${WS_NAME}



FROM osrf/ros:humble-desktop-full
# Differences: desktop vs. desktop-full:
# https://www.ros.org/reps/rep-2001.html#humble-hawksbill-may-2022-ongoing


# Use bash shell as default instead of sh shell, for convenience.
SHELL ["/bin/bash", "-c"]

# Only during build:
ARG DEBIAN_FRONTEND=noninteractive



# Get dependencies:

RUN apt update && \
    apt upgrade -y && \
    apt install -y --no-install-recommends \
        python3-dev \
        python3-pip \
        python3-setuptools \
        git \
        g++ \
        wget \
        make \
        nano \
        cmake \
        build-essential \
        linux-headers-5.19.0-50-generic

# For Python API:
RUN python3 -m pip install --upgrade pip



# Install librealsense:

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# 'software-properties-common' for 'add-apt-repository':
RUN apt install -y software-properties-common && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
    apt install -y \
        librealsense2-dkms \
        librealsense2-utils \
        librealsense2-dev

RUN apt-get update && apt-get install -y libpcl-dev


# ROS setup:

ARG WS_NAME=ros2_ws


# Copy the LOP2 package:
COPY ./lop2/ /${WS_NAME}/src/lop2
COPY ./lop2_interfaces/ /${WS_NAME}/src/lop2_interfaces
COPY ./pointcloud_processing/ /${WS_NAME}/src/pointcloud_processing

# ROS-RealSense-wrapper install:
WORKDIR /${WS_NAME}/src
# Release 4.51.1 from 13.09.2022:
RUN git clone https://github.com/IntelRealSense/realsense-ros.git \
        --branch 4.51.1 \
        --depth 1

# - rosdep init throws an error if it is already initialised, this is ignored.
# - Setuptools version 58.2.0 is the last version that works with ROS 2 python packages without any warnings
# because it is the last version that supports the old installation method, "python setup.py install."
# This method has been deprecated and replaced by newer, standards-based tools, such as pip and ament.
# Setup ROS workspace
WORKDIR /${WS_NAME}
RUN apt install python3-rosdep -y && \
    (rosdep init || true) && rosdep update --include-eol-distros && \
    python3 -m pip install setuptools==58.2.0

# Source and build in the same layer
RUN /bin/bash -c ". /opt/ros/humble/setup.bash && \
    rosdep install -i -y --from-path src --rosdistro humble --skip-keys='librealsense2 lop2_interfaces PCL' && \
    colcon build"

# Add setup file to shell:
RUN (echo; echo 'source /opt/ros/humble/setup.bash') >> ~/.bashrc

# Clear cache:
RUN rm -rf /var/lib/apt/lists/* /tmp/*


RUN apt-get update && apt-get install -y \
    obs-studio \
    xvfb \
    x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Run Xvfb and x11vnc
CMD Xvfb :99 -screen 0 1024x768x16 & export DISPLAY=:99 && x11vnc -display :99 



WORKDIR /${WS_NAME}
