#!/bin/bash

docker run \
  -e SDL_VIDEODRIVER=x11 \
  -e DISPLAY=$DISPLAY\
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -p 2000-2002:2000-2002 \
  -it \
  --rm \
  --name="CarlaDocker" \
  --gpus all \
  carlasim/carla:0.9.11 ./CarlaUE4.sh -opengl
