#/usr/bin/bash

# Environment setup
mkdir -p build
cd build
# Compilation
cmake ..
make -j$nproc
# Testing
# make test