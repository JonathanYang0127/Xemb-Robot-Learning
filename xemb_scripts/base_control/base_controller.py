import serial
import time

class MDCRobot:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, default_speed=200):
        self.port = port
        self.baudrate = baudrate
        self.default_speed = default_speed
        self.ser = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            time.sleep(2)  # Let controller initialize
            print(f"[✓] Connected to MDC2460 on {self.port}")
        except Exception as e:
            print(f"[X] Failed to connect: {e}")
            raise

    def send_command(self, cmd):
        full_cmd = f"{cmd}\r".encode()
        self.ser.write(full_cmd)

    def drive(self, speed=None, turn=0):
        """
        Drive forward/backward and turn.
        :param speed: -1000 to 1000 (forward/backward power)
        :param turn: -1000 to 1000 (right/left turning bias)
        """
        if speed is None:
            speed = self.default_speed
        self.send_command(f'!G 1 {speed}')  # Drive (M1)
        self.send_command(f'!G 2 {turn}')   # Turn  (M2)

    def stop(self):
        self.drive(0, 0)

    def forward(self, duration=1.0):
        self.drive(speed=self.default_speed, turn=0)
        time.sleep(duration)
        self.stop()

    def backward(self, duration=1.0):
        self.drive(speed=-self.default_speed, turn=0)
        time.sleep(duration)
        self.stop()

    def turn_left(self, strength=None, duration=1.0):
        if strength is None:
            strength = self.default_speed
        start = time.time()
        while time.time() - start < duration:
            self.drive(speed=0, turn=strength)
            time.sleep(0.1)
        self.stop()

    def turn_right(self, strength=None, duration=1.0):
        if strength is None:
            strength = self.default_speed
        start = time.time()
        while time.time() - start < duration:
            self.drive(speed=0, turn=-strength)
            time.sleep(0.1)
        self.stop()

    def shutdown(self):
        self.stop()
        if self.ser:
            self.ser.close()
            print("[✓] Serial connection closed.")

