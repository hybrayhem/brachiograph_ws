#!/bin/bash

# Entrypoint for the container
source /root/brachiograph_ws/devel/setup.bash
roslaunch brachiograph_simulation simulation.launch port:=3800 #port:=55000