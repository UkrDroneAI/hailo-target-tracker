#! /usr/bin/env bash

cd /home/udaiadmin/Projects/hailo-target-tracker
source setup_env.sh
python basic_pipelines/detection.py --hef-path resources/yolov8-car-1-class.hef --input rpi