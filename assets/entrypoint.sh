#!/bin/bash
set -e

# Source environment
source ~/.bashrc

# Source workspace, if it exists
if [ -f ~/pumas_nav_ws/install/setup.bash ]; then
  source ~/pumas_nav_ws/install/setup.bash
fi

exec "$@"