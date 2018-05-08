#!/bin/sh

ROOT=$(pwd)

# Build dependencies
mkdir -p dependencies
cd dependencies
cmake ../../dependencies -DCMAKE_INSTALL_PREFIX=$ROOT/dependencies-prefix
cmake --build .
cd ..

# Export the lib directory of the dependencies as a library path
export LD_LIBRARY_PATH=$ROOT/dependencies-prefix/lib

# Build components
mkdir -p main
cd main
cmake ../.. -DCMAKE_PREFIX_PATH=$ROOT/dependencies-prefix
cmake --build .

