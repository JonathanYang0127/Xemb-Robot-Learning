from base_controller import MDCRobot
import time

def main():
    # Initialize and connect to the robot
    robot = MDCRobot('/dev/tty_base')
    robot.connect()

    print("Driving forward for 0.5 seconds...")
    #robot.forward(duration=0.2)

    try:
        #print("Turning left for 0.5 seconds...")
        #robot.turn_right(duration=-5)

        #print("Turning right for 0.5 seconds...")
        #robot.turn_right(duration=0.5)

        #print("Driving backward for 0,5 seconds...")
        robot.backward(duration=0.5)

        print("Stopping...")
        robot.stop()

    finally:
        robot.shutdown()

if __name__ == '__main__':
    main()
