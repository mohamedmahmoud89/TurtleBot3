#!/bin/bash

echo "enter trace name"
read name
echo "replaying $name.bag"
rosbag play -l ../recordings/$name.bag
