#!/usr/bin/env bash

mkdir -p dist
g++ \
  -std=c++1z \
  -lopencv_imgproc \
  -lopencv_highgui \
  -lopencv_core \
    video-led-driver.cpp \
    rpi_ws281x/libws2811.a \
  -o dist/video-led-driver
