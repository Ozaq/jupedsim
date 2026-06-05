#!/bin/bash
set -ex
mkdir build && cd build
cmake  .. -DCMAKE_PREFIX_PATH=/opt/deps -DWITH_FORMAT=ON -DWITH_TIDY=ON

cmake --build . --target check-format -- VERBOSE=1
cmake --build . --target check-tidy -- VERBOSE=1

cd ..

ruff check --output-format full
ruff format --diff
