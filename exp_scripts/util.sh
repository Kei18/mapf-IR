#!/bin/bash

set -e

function getDate() {
    if [ "$(uname)" == "Darwin" ]; then
        gdate +%Y-%m-%d_%H-%M-%S-%2N
    else  # linux
        date +%Y-%m-%d_%H-%M-%S-%2N
    fi
}
