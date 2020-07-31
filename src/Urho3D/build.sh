#!/bin/bash
cmake -DCMAKE_BUILD_TYPE=Debug -DVIDEO_WAYLAND:BOOL=OFF .
make -j9
