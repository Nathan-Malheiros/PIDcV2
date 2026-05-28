cmake_minimum_required(VERSION 3.16)
# Force target before project.cmake is loaded so "idf.py build" never
# defaults to esp32 even when sdkconfig is absent.
set(IDF_TARGET "esp32s3")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ball_balancer)
