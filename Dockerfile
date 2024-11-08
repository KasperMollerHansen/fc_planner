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