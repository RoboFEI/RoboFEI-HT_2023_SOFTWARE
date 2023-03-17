#!/bin/bash

cd ~/RoboFEI-HT_2023_SOFTWARE
sudo rm -rf build install log runs
colcon build --symlink-install
source install/setup.bash
