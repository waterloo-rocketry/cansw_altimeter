#!/bin/sh
set -x
${CLANG_TIDY:-clang-tidy} src/apps/*/*.cpp src/apps/*/*.hpp src/drivers/*/*.cpp src/drivers/*/*.hpp
