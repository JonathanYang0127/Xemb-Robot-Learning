sudo tailscale up
sudo tailscale status

dont forget to kill dynamixel wizard after using it; only one can connect at any given time

source ~/interbotix_ws/devel/setup.bash
cd ~/interbotix_ws/src/xemb_aloha/xemb_scripts/ 
roslaunch aloha master_both.launch 

source ~/interbotix_ws/devel/setup.bash
conda activate aloha
cd ~/interbotix_ws/src/xemb_aloha/xemb_scripts/
python relay_master_joints_full.py

