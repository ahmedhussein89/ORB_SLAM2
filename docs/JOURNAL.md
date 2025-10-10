# ORB_SLAM2

The popose of this repo is to refactor the code to be modular and follow the best practice.

## Create baseline

In this stage I need to create a Dockerfile to build the sourcecode. The code was created on ubuntu 16.04 which is so old.
I chose to build the code using Ubuntu 18.04 which gives more beverage. So I created a dockerfile which is based on Ubuntu 18.04.
I installed the minimual requiremnts:

- build-essential
- cmake
- opencv
- boost `boost::bind` -> can be removed
- glew for Pangolin
- Pangolin `installed from source code`

I copied the sourcecode inside the container and build it while building the image.
I faced missing `usleep` function which is solved by adding `unistd.h` header.

You can find the the base docker file in the following path `tools/Dockerfile`.

To build the docker image you can run the following command

`docker build . -t orb_slam2 -f tools/Dockerfile`

To run the docker

```bash
xhost + # Important or the script will exec will exit
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v .:/tmp/ORB_SLAM2:rw \
  -v ~/.Xauthority:/root/.Xauthority:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $DATASET:/tmp/Dataset \
  orb_slam2 \
  --example mono_euroc \
  --dataset /tmp/Dataset/euroc/machine_hall/mav0/cam0/data \
  --settings MH01
```

## Download dataset

I created a python script to download EuRoC and Kitti dataset. You can find it in the `tools/slam_downloader.py`
I can be used as the following

```bash
python tools/slam_downloader.py -d all -o .
```

## Acceptance test

I want to create a script to trace CPU, Memory, stdout and stderr while running the example.

## Evaluation script


