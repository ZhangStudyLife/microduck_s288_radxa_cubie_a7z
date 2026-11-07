#!/bin/sh
set -eu
cd "$(dirname "$0")"
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
g++ -O1 -g -std=c++17 -Wall -Wextra -Wconversion -pthread -fsanitize=undefined -Wl,--wrap=fsync test_joints.cpp ../s288/s288.cpp -o "$work/test_joints"
cd "$work"
./test_joints
