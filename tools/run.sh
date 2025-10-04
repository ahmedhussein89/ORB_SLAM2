#!/usr/bin/env bash

docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v .:/home/ORB_SLAM2:rw \
  -w /home/orb_slam2 \
  -v ~/.Xauthority:/root/.Xauthority:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ahussein/Documents/sourcecode/archive/Dataset:/tmp/Dataset \
  orb_slam2 \
  /opt/orb_slam2/bin/mono_euroc /opt/orb_slam2/Vocabulary/ORBvoc.txt /opt/orb_slam2/examples/Monocular/EuRoC.yaml /tmp/Dataset/euroc/machine_hall/mav0/cam0/data /opt/orb_slam2/examples/Monocular/EuRoC_TimeStamps/MH01.txt
