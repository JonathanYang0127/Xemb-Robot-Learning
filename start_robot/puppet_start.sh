#!/bin/bash

# Function to open a new terminal tab and run a command
open_tab_and_run() {
    # Detect which terminal emulator is being used
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --tab -- bash -c "$1; exec bash"
    elif command -v xfce4-terminal &> /dev/null; then
        xfce4-terminal --tab --command="bash -c '$1; exec bash'"
    elif command -v konsole &> /dev/null; then
        konsole --new-tab -e bash -c "$1; exec bash"
    elif command -v terminator &> /dev/null; then
        terminator -e "bash -c '$1; exec bash'" &
    else
        echo "Unsupported terminal. Please install gnome-terminal, xfce4-terminal, konsole, or terminator."
        exit 1
    fi
}

# Tab 1: Run sunshine
open_tab_and_run "echo 'Starting sunshine...'; sunshine"

# Give a moment for the first tab to initialize
sleep 1

# Tab 2: Source setup.bash and run roslaunch
open_tab_and_run "echo 'Launching ROS nodes...'; source ~/interbotix_ws/devel/setup.bash && roslaunch aloha puppet_both.launch"

# Give a moment for the second tab to initialize
sleep 1

# Tab 3: Activate conda environment and run the puppet follow script
open_tab_and_run "echo 'Activating conda environment and starting puppet follow script...'; source ~/miniconda3/etc/profile.d/conda.sh && conda activate aloha && cd ~/interbotix_ws/src/xemb_aloha/xemb_scripts && python puppet_follow_full_smooth.py"

echo "All tabs have been launched!"

