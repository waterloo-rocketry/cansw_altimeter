#!/bin/sh

${CLANG_FORMAT:-clang-format} -i --style=file:third_party/rocketlib/.clang-format src/apps/*/*.cpp src/apps/*/*.hpp src/drivers/*/*.cpp src/drivers/*/*.hpp
