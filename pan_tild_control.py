import time
import board
import busio
from adafruit_pca9685 import PCA9685
from math import floor

# Initialize I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50

# Servo pulse range
SERVO_MIN = 102  # ~0.5ms
SERVO_MAX = 512  # ~2.5ms

def set_servo_angle(channel, angle):
    pulse = floor(((angle + 90) / 180) * (SERVO_MAX - SERVO_MIN) + SERVO_MIN)
    pca.channels[channel].duty_cycle = pulse << 4

try:
    for angle in range(-90, 91, 10):
        set_servo_angle(0, angle)  # Pan
        set_servo_angle(1, angle)  # Tilt
        time.sleep(0.5)
    set_servo_angle(0, 0)
    set_servo_angle(1, 0)
except KeyboardInterrupt:
    set_servo_angle(0, 0)
    set_servo_angle(1, 0)
    pca.deinit()