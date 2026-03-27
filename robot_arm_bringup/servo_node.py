#!/usr/bin/env python3
"""
servo_node.py
=============
Subscribes to /servo_cmd (robot_arm_bringup/ServoCmd) and drives 4 arm
servos through the PCA9685 on the Waveshare Motor+Servo HAT.

Channel mapping (configurable via ROS params):
  channel_base      = 0   (base rotation)
  channel_shoulder  = 1   (shoulder)
  channel_elbow     = 2   (elbow)
  channel_gripper   = 3   (gripper)

PCA9685 I2C default address: 0x40
PWM frequency: 50 Hz  (standard hobby servo)
Pulse range:   500 us – 2500 us  (adjust min/max params if servos differ)
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from robot_arm_bringup.msg import ServoCmd

try:
    from adafruit_pca9685 import PCA9685
    import board
    import busio
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def angle_to_pulse(angle_deg: float, min_us: float, max_us: float,
                   pwm_freq: float) -> int:
    """Convert an angle in degrees [0,180] to a PCA9685 12-bit duty value."""
    angle_deg = max(0.0, min(180.0, angle_deg))
    pulse_us = min_us + (max_us - min_us) * (angle_deg / 180.0)
    # PCA9685 resolution: 4096 steps per period
    period_us = 1_000_000.0 / pwm_freq
    return int((pulse_us / period_us) * 4096)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ServoNode(Node):

    CHANNEL_NAMES = ['base', 'shoulder', 'elbow', 'gripper']

    def __init__(self):
        super().__init__('servo_node')

        # --- Parameters -------------------------------------------------------
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('pwm_frequency', 50.0)
        self.declare_parameter('pulse_min_us', 500.0)   # 0 deg
        self.declare_parameter('pulse_max_us', 2500.0)  # 180 deg
        self.declare_parameter('channel_base', 0)
        self.declare_parameter('channel_shoulder', 1)
        self.declare_parameter('channel_elbow', 2)
        self.declare_parameter('channel_gripper', 3)

        self._freq    = self.get_parameter('pwm_frequency').value
        self._min_us  = self.get_parameter('pulse_min_us').value
        self._max_us  = self.get_parameter('pulse_max_us').value
        self._channels = [
            self.get_parameter(f'channel_{n}').value
            for n in self.CHANNEL_NAMES
        ]

        # --- Hardware init ----------------------------------------------------
        self._pca = None
        if HARDWARE_AVAILABLE:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                addr = self.get_parameter('i2c_address').value
                from adafruit_pca9685 import PCA9685
                self._pca = PCA9685(i2c, address=addr)
                self._pca.frequency = self._freq
                self.get_logger().info(
                    f'PCA9685 initialised at 0x{addr:02X}, {self._freq} Hz')
            except Exception as e:
                self.get_logger().error(f'PCA9685 init failed: {e}')
        else:
            self.get_logger().warn(
                'adafruit_pca9685 / board not found — running in DRY-RUN mode')

        # Current positions (degrees), start centred
        self._current = [90.0, 90.0, 90.0, 90.0]
        self._set_all(self._current)

        # --- Subscriber -------------------------------------------------------
        self._sub = self.create_subscription(
            ServoCmd, '/servo_cmd', self._on_servo_cmd, 10)

        self.get_logger().info('servo_node ready — listening on /servo_cmd')

    # ------------------------------------------------------------------

    def _set_servo(self, index: int, angle_deg: float):
        """Send a single servo to the requested angle."""
        ch = self._channels[index]
        duty = angle_to_pulse(angle_deg, self._min_us, self._max_us, self._freq)
        if self._pca:
            self._pca.channels[ch].duty_cycle = duty << 4  # 12-bit → 16-bit
        self.get_logger().debug(
            f'servo[{index}] ch={ch} → {angle_deg:.1f}° (duty={duty})')

    def _set_all(self, angles):
        for i, a in enumerate(angles):
            self._set_servo(i, a)

    def _on_servo_cmd(self, msg: ServoCmd):
        for i in range(4):
            target = float(msg.angles[i])
            if target < 0.0:
                # Negative angle means "no change" — keep current position
                continue
            self._current[i] = target
            self._set_servo(i, target)

    def destroy_node(self):
        if self._pca:
            self._pca.deinit()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
