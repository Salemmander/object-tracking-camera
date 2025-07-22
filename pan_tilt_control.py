import busio
import board
from adafruit_pca9685 import PCA9685
from math import floor

i2c = busio.I2C(board.SCL, board.SDA)

pca = PCA9685(i2c, address=0x40)

pca.frequency = 50

SERVO_PAN_MIN = 102 << 4
SERVO_PAN_MAX = 512 << 4

SERVO_TILT_MIN = 330 << 4
SERVO_TILT_MAX = 550 << 4
# PAN and TILT for the channel param in set_servo_angle
TILT = 0
PAN = 1


def set_servo_angle(channel, angle):
    angle = max(-90, min(90, angle))
    if channel == PAN:
        pulse = floor(
            ((angle + 90) / 180) * (SERVO_PAN_MAX - SERVO_PAN_MIN) + SERVO_PAN_MIN
        )
    elif channel == TILT:
        pulse = floor(
            ((angle + 90) / 180) * (SERVO_TILT_MAX - SERVO_TILT_MIN) + SERVO_TILT_MIN
        )
    pca.channels[channel].duty_cycle = pulse


def get_angle(channel):
    pulse = pca.channels[channel].duty_cycle
    if channel == PAN:
        angle = ((pulse - SERVO_PAN_MIN) / (SERVO_PAN_MAX - SERVO_PAN_MIN)) * 180 - 90
    elif channel == TILT:
        angle = (
            (pulse - SERVO_TILT_MIN) / (SERVO_TILT_MAX - SERVO_TILT_MIN)
        ) * 180 - 90

    return angle


def cleanup():
    pca.channels[PAN].duty_cycle = 0
    pca.channels[TILT].duty_cycle = 0
    pca.deinit()
    i2c.deinit()
