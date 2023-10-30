#!/bin/bash
set -e

# setup ros environment
source "/home/visavis_ws/devel/setup.bash" --
exec "$@"