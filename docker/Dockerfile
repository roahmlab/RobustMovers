# Use an official Ubuntu runtime as a parent image
FROM ubuntu:22.04

# Set the working directory in the container
WORKDIR /app

SHELL ["/bin/bash", "-c"]

# Install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    gdb \
    valgrind \
    libboost-all-dev \
    libgsl-dev \
    git \
    patch \
    wget \
    pkg-config \
    liblapack-dev \
    libmetis-dev \
    python3.10 \
    python3-pip \
    iputils-ping \
    net-tools \
    netcat \
    screen \
    lsb-release \
    gnupg2 \
    curl \
    libyaml-cpp-dev \
    xvfb \
    libeigen3-dev \
    liburdfdom-dev

    # install python packages
RUN pip install --upgrade pip && \
    pip install scipy matplotlib pyyaml pybullet==3.2.5 glfw==2.6.2 pyopengl==3.1.7 numpy==1.25.2 nanobind pygccxml pyplusplus

#     # Install Eigen-3.3.7 dependencies
# RUN --mount=type=cache,target=/var/cache/apt \
#     --mount=type=bind,source=docker/scripts/install-eigen.sh,target=/tmp/install-eigen.sh \
#     bash /tmp/install-eigen.sh

    # Install pinochio dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=bind,source=docker/scripts/install-pinocchio-dependencies.sh,target=/tmp/install-pinocchio-dependencies.sh \
    bash /tmp/install-pinocchio-dependencies.sh
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=bind,source=docker/scripts/install-pinocchio.sh,target=/tmp/install-pinocchio.sh \
    bash /tmp/install-pinocchio.sh

ENV PATH /opt/openrobots/bin:/usr/local/bin:/usr/bin
ENV PKG_CONFIG_PATH /opt/openrobots/lib/pkgconfig
ENV LD_LIBRARY_PATH /usr/lib:/usr/lib/x86_64-linux-gnu:/opt/openrobots/lib:/usr/local/lib:/usr/local/share
ENV PYTHONPATH /opt/openrobots/lib/python3.10/site-packages
ENV CMAKE_PREFIX_PATH /usr:/opt/openrobots:/usr/local:/usr/local/share

# Set the entry point to VSCode's default command
CMD ["/bin/bash"]

USER $USERNAME