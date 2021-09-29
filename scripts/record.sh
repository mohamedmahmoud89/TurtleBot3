#!/bin/bash

echo "enter trace name"

read var 

echo "recording into $var.bag"

rosbag record -O ../recordings/$var.bag /odom /ldsScan
