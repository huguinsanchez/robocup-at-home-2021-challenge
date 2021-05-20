#!/bin/bash
set -e

# setup ros environment
source "/workspace/devel/setup.bash"
exec "$@"