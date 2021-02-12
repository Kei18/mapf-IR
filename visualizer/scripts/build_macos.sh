#!/bin/bash
set -e
cd `dirname $0`/../../third_party/openFrameworks/
cp -f ../../visualizer/scripts/download_libs.sh ./scripts/dev/download_libs.sh
bash ./scripts/osx/download_libs.sh -v 0.11.0
cd ../../visualizer/
make build
cd ..
chmod +x ./visualize.sh
