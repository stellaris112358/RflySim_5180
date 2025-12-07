#!/bin/bash

unzip -o src.zip -d /home


cd  /home/ego-planner-swarm 
chmod 777 -R *

catkin_make
