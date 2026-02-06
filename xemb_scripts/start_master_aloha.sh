#!/bin/bash

SESSION="aloha"

# Start new tmux session detached
tmux new-session -d -s $SESSION

# Pane 1: ROS launch
tmux send-keys -t $SESSION:0 'source ~/interbotix_ws/devel/setup.bash' C-m
tmux send-keys -t $SESSION:0 'roslaunch aloha master_both.launch'

# Split window horizontally for Pane 2
tmux split-window -h -t $SESSION:0

# Pane 2: Conda, cd, etc.
tmux send-keys -t $SESSION:0.1 'source ~/interbotix_ws/devel/setup.bash' C-m
tmux send-keys -t $SESSION:0.1 'conda activate aloha' C-m
tmux send-keys -t $SESSION:0.1 'cd ~/interbotix_ws/src/xemb_aloha/xemb_scripts' C-m
# tmux send-keys -t $SESSION:0.1 'python relay_master_joints_full_direct.py --no-move-left --gripper-lock-left'
tmux send-keys -t $SESSION:0.1 'python relay_master_joints_full_direct.py'

# Attach to the session
tmux attach-session -t $SESSION 
