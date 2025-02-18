#!/usr/bin/env bash
#
#
# KumarRobotics DCIST Master Image based on Open Robotics Image
#
# * * * *
#
# Copyright (C) 2018 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -eo pipefail

# Check that the current user has UID 1000.
if [ $(id -u) -ne 1000 ]
then
  echo "ERROR: This script must be run with UID and GID of 1000."
  echo "       Current UID: $(id -u), current GID: $(id -g)"
  exit 1
fi

if [ $# -lt 1 ]
then
    echo "Usage: $0 <docker image> [<dir with workspace> ...]"
    exit 1
fi

IMG="kumarrobotics/$(basename $1)"

# Get the current folder for the docker run command
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
USER_WS="$CURR_DIR/ws"
DATA_DIR="$CURR_DIR/data"
ROS_DIR="$CURR_DIR/.ros_docker"
BASHRC_HOST="$CURR_DIR/bashrc"

# Make sure processes in the container can connect to the x server
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    touch $XAUTH
fi
xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
if [ -n "$xauth_list" ]
then
  echo "$xauth_list" | xauth -f $XAUTH nmerge -
fi
chmod a+r $XAUTH

# Print in purple
echo -e "\033[1;35mRUNNING DOCKER IMAGE: $1\033[0m"
echo -e "\033[1;35mUSER WORKSPACE: \033[0m$USER_WS"
echo -e "\033[1;35mDATA DIR: \033[0m$DATA_DIR"
echo -e "\033[1;35mROS DIR: \033[0m$ROS_DIR"
echo -e "\033[1;35mBASHRC_HOST: \033[0m$BASHRC_HOST"

# get GID of input group
INPUT_GID=$(getent group input | cut -d: -f3)

# Mount extra volumes if needed.
docker run --gpus all \
  -u 1000 \
  -it \
  --workdir /home/dcist \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev:/dev" \
  -v "/media/$USER:/media/dcist" \
  -v "/home/$USER/.bash_history:/home/dcist/.bash_history" \
  --network host \
  -h dcist \
  --add-host dcist:127.0.0.1 \
  --add-host dcist:192.168.8.100 \
  -v "$DATA_DIR:/home/dcist/data" \
  -v "$ROS_DIR:/home/dcist/.ros" \
  -v "$BASHRC_HOST:/home/dcist/.bashrc_host" \
  --rm \
  --security-opt seccomp=unconfined \
  --group-add=dialout \
  --group-add $INPUT_GID \
  "$IMG"