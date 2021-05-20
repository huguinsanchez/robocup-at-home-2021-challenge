#!/bin/bash
set -e

# setup ros environment
#source "/opt/ros/melodic/setup.bash"
source "/workspace/devel/setup.bash"
exec "$@"