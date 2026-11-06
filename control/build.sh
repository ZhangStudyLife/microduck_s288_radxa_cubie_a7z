#!/bin/sh
set -eu
cd "$(dirname "$0")"
g++ -O2 -std=c++17 -Wall -Wextra -Wconversion -pthread robotd.cpp ../s288/s288.cpp -o robotd
