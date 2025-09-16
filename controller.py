import busio
import board
from adafruit_pca9685 import PCA9685
from math import floor


class PanTiltController:
    SERVO_PAN_MIN = 102 << 4
    SERVO_PAN_MAX = 512 << 4

    SERVO_TILT_MIN = 200 << 4
    SERVO_TILT_MAX = 405 << 4
    # PAN and TILT for the channel param in set_servo_angle
    TILT = 0
    PAN = 1

    def __init__(self, ic2_address=0x40, frequency=50):
        self.i2c = busio.I2C(board.SCL, board.SDA)

        self.pca = PCA9685(self.i2c, address=ic2_address)

        self.pca.frequency = frequency

    def set_servo_angle(self, channel, angle):
        angle = max(0, min(180, angle))
        if channel == self.PAN:
            pulse = floor(
                (angle / 180) * (self.SERVO_PAN_MAX - self.SERVO_PAN_MIN)
                + self.SERVO_PAN_MIN
            )
        elif channel == self.TILT:
            pulse = floor(
                (angle / 180) * (self.SERVO_TILT_MAX - self.SERVO_TILT_MIN)
                + self.SERVO_TILT_MIN
            )
        self.pca.channels[channel].duty_cycle = pulse

    def get_angle(self, channel):
        pulse = self.pca.channels[channel].duty_cycle
        if channel == self.PAN:
            angle = (
                (pulse - self.SERVO_PAN_MIN) / (self.SERVO_PAN_MAX - self.SERVO_PAN_MIN)
            ) * 180
        elif channel == self.TILT:
            angle = (
                (pulse - self.SERVO_TILT_MIN)
                / (self.SERVO_TILT_MAX - self.SERVO_TILT_MIN)
            ) * 180

        return angle

    def cleanup(self):
        self.pca.channels[self.PAN].duty_cycle = 0
        self.pca.channels[self.TILT].duty_cycle = 0
        self.pca.deinit()
        self.i2c.deinit()
