#!/bin/bash

# Start the LOP2 Docker container with support for graphics/GPU.

# Warning: '--privileged' is used.


# Argument to use only CPU, no GPU:
use_cpu=$( [ "$1" = '--cpu' ] && echo 'true' || echo 'false' )

gpu_parameters='--gpus all
--env NVIDIA_VISIBLE_DEVICES=all
--env NVIDIA_DRIVER_CAPABILITIES=all'

additional_parameters=$( [ "$use_cpu" = 'true' ] && echo '' || echo "$gpu_parameters" )


container_name='lop2-custom'
image_name='lop2-custom:latest'


sudo xhost +si:localuser:root

# Delete existing container with chosen name: 
docker container rm "$container_name" > /dev/null 2>&1

# docker run -it \
# 	$additional_parameters \
# 	-e DISPLAY=$DISPLAY \
# 	--env DISPLAY \
# 	--env QT_X11_NO_MITSHM=1 \
# 	-v /tmp/.X11-unix:/tmp/.X11-unix \
# 	-v /dev:/dev \
# 	--net=host \
# 	--privileged \
# 	--device=/dev/video0:/dev/video0 \
# 	--name "$container_name" \
# 	"$image_name" \
# 	/bin/bash


docker run -it \
	$additional_parameters \
	-e DISPLAY=:99 \
	--env QT_X11_NO_MITSHM=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /dev:/dev \
	--net=host \
	--privileged \
	--device=/dev/video0:/dev/video0 \
	--name "$container_name" \
	"$image_name" \
	/bin/bash -c "Xvfb :99 -screen 0 1024x768x16 & export DISPLAY=:99 && /bin/bash"