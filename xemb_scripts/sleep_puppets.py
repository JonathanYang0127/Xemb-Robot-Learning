from interbotix_xs_modules.arm import InterbotixManipulatorXS
from robot_utils import move_arms, torque_on, torque_off
import argparse

def main():
    parser = argparse.ArgumentParser(description="Put puppets to sleep position.")
    parser.add_argument('--torque-off', action='store_true', help='Torque off at the end')
    args = parser.parse_args()

    puppet_bot_left = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=f'puppet_left', init_node=True)
    puppet_bot_right = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=f'puppet_right', init_node=False)
    all_bots = [puppet_bot_left, puppet_bot_right]
    for bot in all_bots:
        torque_on(bot)

    puppet_sleep_position = (0, -1.7, 1.55, 0.12, 0.65, 0)
    move_arms(all_bots, [puppet_sleep_position] * 2, move_time=2)

    if args.torque_off:
        torque_off(puppet_bot_left)
        torque_off(puppet_bot_right)

if __name__ == '__main__':
    main()
