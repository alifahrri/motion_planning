#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

docker run -it -v $DIR/../:/app/motion_planning_ws/src/motion_planning/ ros:kinetic-ros-base bash