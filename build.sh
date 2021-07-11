#/usr/bin/bash

# Environment setup
mkdir -p build
cd build
# Compilation
cmake .. -DCMAKE_BUILD_TYPE=DEBUG #RELWITHDEBINFO
make -j$nproc
make install
# Testing
# make test