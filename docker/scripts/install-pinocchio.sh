git clone https://github.com/roahmlab/pinocchio.git
cd pinocchio
git checkout devel
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/openrobots
make -j4
make install
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH