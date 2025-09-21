docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v ~/.Xauthority:/root/.Xauthority:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ahussein/Downloads/Dataset/:/tmp/Dataset \
  -w /workspace/ORB_SLAM2/Examples/Monocular \
  orb_slam2 \
  bash
# ./mono_euroc ../../Vocabulary/ORBvoc.txt EuRoC.yaml /tmp/mav0/cam0/data EuRoC_TimeStamps/MH01.txt 
