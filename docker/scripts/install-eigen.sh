#!/bin/bash

EIGEN_VERSION="3.3.7"
EIGEN_URL="https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz"

INSTALL_DIR="/usr/local/eigen-${EIGEN_VERSION}"

echo "Downloading Eigen ${EIGEN_VERSION}..."
wget ${EIGEN_URL} -O eigen-${EIGEN_VERSION}.tar.gz

echo "Extracting Eigen ${EIGEN_VERSION}..."
tar -xzf eigen-${EIGEN_VERSION}.tar.gz

cd eigen-${EIGEN_VERSION}
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
make install
