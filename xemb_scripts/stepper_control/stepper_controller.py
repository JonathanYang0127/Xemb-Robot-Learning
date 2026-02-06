import serial
import time

class StepperController:
    def __init__(self, port="/dev/tty_stepper", baudrate=9600, timeout=1):
        print(f"Connecting to Arduino on {port}...")
        self.arduino = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for Arduino to reset
        print("Connected.")

    def enable(self):
        self._send_command('e')

    def disable(self):
        self._send_command('s')

    def forward(self):
        #TODO: Reverse
        self._send_command('b')

    def backward(self):
        #TODO Reverse
        self._send_command('f')

    def stop(self):
        self._send_command('x')

    def close(self):
        print("Closing connection.")
        self.arduino.close()

    def _send_command(self, command):
        if self.arduino.is_open:
            print(f"Sending: {command}")
            self.arduino.write(command.encode())
        else:
            print("Serial port not open.")


