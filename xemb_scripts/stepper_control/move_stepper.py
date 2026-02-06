# stepper_cli_control.py

import argparse
from stepper_controller import StepperController

def main():
    parser = argparse.ArgumentParser(description="Control stepper motor via command line.")
    parser.add_argument(
        "direction", choices=["up", "down"],
        help="Direction to move the motor: 'up' for forward, 'down' for backward"
    )
    parser.add_argument(
        "--port", default="/dev/tty_stepper",
        help="Serial port to which the stepper is connected (default: /dev/tty_stepper)"
    )
    args = parser.parse_args()

    motor = StepperController(port=args.port)
    motor.enable()

    try:
        if args.direction == "up":
            motor.forward()
            input("Motor moving up... press ENTER to stop\n")
        else:
            motor.backward()
            input("Motor moving down... press ENTER to stop\n")
        motor.stop()
    finally:
        motor.disable()
        motor.close()

if __name__ == "__main__":
    main()

