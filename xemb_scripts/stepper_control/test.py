from stepper_controller import StepperController
import time

motor = StepperController(port="/dev/tty_stepper")  # Change port if needed

motor.enable()
motor.forward()

input("Motor spinning forward... press ENTER to stop\n")
motor.stop()

motor.backward()
input("Motor spinning backward... press ENTER to stop\n")
motor.stop()

motor.disable()
motor.close()

