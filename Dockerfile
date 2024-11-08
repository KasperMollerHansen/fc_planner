FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get upgrade -y
    
RUN apt-get install -y \
    python3-pip\
    python3-dev\
    libgl1 \
    libglib2.0-0 \
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
    ccache \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update 

RUN python3 -m pip install --no-cache-dir open3d

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