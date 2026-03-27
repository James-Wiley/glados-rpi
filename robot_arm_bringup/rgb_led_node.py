#!/usr/bin/env python3
"""
rgb_led_node.py
===============
Subscribes to /rgb_led (robot_arm_bringup/RgbLed) and drives a single
WS2812B (NeoPixel) over SPI (/dev/spidev0.0).

Using SPI instead of PWM because Ubuntu 22.04 on the Pi Zero 2W does not
expose the PWM-backed /dev/mem interface that rpi_ws281x requires.  SPI
produces a stable 800 kHz-equivalent waveform without any kernel module
gymnastics.

Protocol encoding
-----------------
WS2812B expects GRB order at 800 kHz.  Each bit is encoded as a SPI byte:
  1-bit → 0b11111000  (0xF8)  high ~625 ns, low ~250 ns
  0-bit → 0b11000000  (0xC0)  high ~250 ns, low ~625 ns
At SPI clock = 6.4 MHz one SPI byte ≈ 1.25 µs → exactly one WS2812B bit.
Reset: ≥50 µs of low = send 40+ zero bytes after the pixel data.

Hardware setup
--------------
  SPI MOSI  → GPIO 10 (pin 19) → WS2812B data-in
  3.3 V → 5 V level shifter recommended (74AHCT125 or equivalent).
  Enable SPI:  add  dtparam=spi=on  to /boot/firmware/config.txt, reboot.
  Permissions: sudo usermod -aG spi $USER   (then log out / back in)

Install dependency:
    pip install spidev

ROS params:
  spi_bus      (int, default 0)    SPI bus number
  spi_device   (int, default 0)    chip-select (CE0)
  spi_speed_hz (int, default 6400000)
  led_count    (int, default 1)
"""

import rclpy
from rclpy.node import Node

from robot_arm_bringup.msg import RgbLed

try:
    import spidev
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False

# SPI bytes representing a WS2812B logical 1 and 0
_BIT1 = 0xF8
_BIT0 = 0xC0
_RESET_BYTES = bytes(40)   # ≥50 µs low = reset pulse


def _encode_pixel(r: int, g: int, b: int) -> bytes:
    """Encode one WS2812B pixel (GRB order) into 24 SPI bytes."""
    buf = bytearray(24)
    for i, byte_val in enumerate((g, r, b)):
        for bit in range(8):
            buf[i * 8 + bit] = _BIT1 if (byte_val & (0x80 >> bit)) else _BIT0
    return bytes(buf)


class RgbLedNode(Node):

    def __init__(self):
        super().__init__('rgb_led_node')

        # --- Parameters -------------------------------------------------------
        self.declare_parameter('spi_bus',      0)
        self.declare_parameter('spi_device',   0)
        self.declare_parameter('spi_speed_hz', 6_400_000)
        self.declare_parameter('led_count',    1)

        bus    = self.get_parameter('spi_bus').value
        device = self.get_parameter('spi_device').value
        speed  = self.get_parameter('spi_speed_hz').value
        self._count = self.get_parameter('led_count').value

        # --- Hardware init ----------------------------------------------------
        self._spi = None
        if HARDWARE_AVAILABLE:
            try:
                self._spi = spidev.SpiDev()
                self._spi.open(bus, device)
                self._spi.max_speed_hz = speed
                self._spi.mode = 0
                self._apply(0, 0, 0)   # start off
                self.get_logger().info(
                    f'WS2812B ready — SPI{bus}.{device} @ {speed//1000} kHz, '
                    f'{self._count} pixel(s)')
            except Exception as e:
                self.get_logger().error(
                    f'SPI init failed: {e}\n'
                    f'Check: dtparam=spi=on in /boot/firmware/config.txt '
                    f'and that your user is in the "spi" group.')
                self._spi = None
        else:
            self.get_logger().warn(
                'spidev not found — running in DRY-RUN mode. '
                'Install with: pip install spidev')

        # --- Subscriber -------------------------------------------------------
        self._sub = self.create_subscription(
            RgbLed, '/rgb_led', self._on_rgb, 10)

        self.get_logger().info('rgb_led_node ready — listening on /rgb_led')

    # ------------------------------------------------------------------

    def _apply(self, r: int, g: int, b: int):
        if self._spi:
            payload = _encode_pixel(r, g, b) * self._count + _RESET_BYTES
            self._spi.xfer2(list(payload))
        self.get_logger().debug(f'LED → R={r} G={g} B={b}')

    def _on_rgb(self, msg: RgbLed):
        self._apply(
            max(0, min(255, int(msg.r))),
            max(0, min(255, int(msg.g))),
            max(0, min(255, int(msg.b))),
        )

    def destroy_node(self):
        if self._spi:
            self._apply(0, 0, 0)
            self._spi.close()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RgbLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
