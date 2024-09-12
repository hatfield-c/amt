#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/jetson/code/ros2_jetpack6_ws/install/setup.bash

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/jetson/miniforge3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/jetson/miniforge3/etc/profile.d/conda.sh" ]; then
        . "/home/jetson/miniforge3/etc/profile.d/conda.sh"
    else
        export PATH="/home/jetson/miniforge3/bin:$PATH"
    fi
fi
unset __conda_setup

if [ -f "/home/jetson/miniforge3/etc/profile.d/mamba.sh" ]; then
    . "/home/jetson/miniforge3/etc/profile.d/mamba.sh"
fi
# <<< conda initialize <<<

mamba activate py3.10
python /home/jetson/amt/Main.py