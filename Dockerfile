# Use the ROS Noetic desktop image
FROM osrf/ros:noetic-desktop-full

# Set environment variables to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update and install dependencies for Python 3.10 and curl
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        curl && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3.10 \
        python3.10-dev \
        python3.10-distutils \
        libgl1 \
        libglib2.0-0 && \
    rm -rf /var/lib/apt/lists/*

# Install pip for Python 3.10 using curl
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10

# Update alternatives to make Python 3.10 the default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 2

# Upgrade PyYAML specifically to avoid uninstall issues
RUN python3 -m pip install --upgrade --ignore-installed pyyaml

# Install Open3D using the new Python 3.10 pip with --ignore-installed option
RUN python3 -m pip install --no-cache-dir --ignore-installed open3d

RUN apt-get update && apt-get upgrade -y
    
RUN apt-get install -y \
    python3-rosdep \
    libboost-all-dev \
    python3-catkin-tools \
    python3-empy \
    cpufrequtils \
    libompl-dev \
    git \
    libpcl-dev \ 
    libeigen3-dev \
    ros-noetic-pcl-conversions \
    libpcap-dev \
    libpng-dev \
    libusb-dev \
    ros-noetic-openni-launch \
    ros-noetic-openni2-launch \
    nano \
    ccache

RUN rosdep update 

# Clone GitHub repo
ARG CACHE_BUST
ARG branch

RUN echo "Branch: $branch"

# Clone only selected branch
RUN echo $CACHE_BUST && git clone -b $branch https://github.com/KasperMollerHansen/fc_planner.git --single-branch 

# Automatically source the ROS setup script and gpufreq 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    echo "sudo cpufreq-set -g performance" >> ~/.bashrc

# Build the FC-Planner package
WORKDIR "fc_planner/FC-Planner"

RUN catkin clean
# RUN catkin config -DCMAKE_BUILD_TYPE=Release
RUN catkin config --cmake-args -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_BUILD_TYPE=Release
RUN . /opt/ros/noetic/setup.sh && \
    catkin build -j4 --cmake-args -Wno-dev

RUN echo "source devel/setup.bash" >> ~/.bashrc 