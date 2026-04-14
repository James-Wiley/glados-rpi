#!/usr/bin/env python3
"""
bt_node.py
==========
Bluetooth LE GATT server node for the Pi Zero 2W robot arm.

Acts as a BLE *peripheral* — the Flutter app connects as central and writes
commands to GATT characteristics.  This node decodes the payloads and
publishes to the ROS2 topics consumed by servo_node and rgb_led_node.

GATT Layout
-----------
Service UUID:  12345678-1234-5678-1234-56789abcdef0   (Robot Arm Service)

  Characteristic: SERVO_CMD_UUID
    uuid:  12345678-1234-5678-1234-56789abcdef1
    props: WRITE / WRITE_WITHOUT_RESPONSE
    payload (JSON):
      {"angles": [90, 45, 120, 10], "speeds": [1.0, 0.5, 1.0, 0.8]}
      - angles: 0-180 deg per servo; -1 = no change
      - speeds: 0.0-1.0 (reserved for future use)

  Characteristic: RGB_LED_UUID
    uuid:  12345678-1234-5678-1234-56789abcdef2
    props: WRITE / WRITE_WITHOUT_RESPONSE
    payload (JSON):
      {"r": 255, "g": 128, "b": 0}

  Characteristic: STATUS_UUID  (notify — robot → phone)
    uuid:  12345678-1234-5678-1234-56789abcdef3
    props: READ / NOTIFY
    payload (JSON, sent every STATUS_INTERVAL_S seconds):
      {"connected": true, "servos": [90, 45, 120, 10]}

Dependencies:
  pip install bleak bless   # bless provides the GATT *server* (peripheral) API
"""

import asyncio
import json
import logging
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from robot_arm_bringup.msg import ServoCmd, RgbLed

# --- BLE imports (bless provides a GATT server / peripheral role) ----------
try:
    from bless import (
        BlessServer,
        BlessGATTCharacteristic,
        GATTCharacteristicProperties,
        GATTAttributePermissions,
    )
    BLE_AVAILABLE = True
except ImportError:
    BLE_AVAILABLE = False


# ---------------------------------------------------------------------------
# UUIDs
# ---------------------------------------------------------------------------
SERVICE_UUID     = '12345678-1234-5678-1234-56789abcdef0'
SERVO_CMD_UUID   = '12345678-1234-5678-1234-56789abcdef1'
RGB_LED_UUID     = '12345678-1234-5678-1234-56789abcdef2'
STATUS_UUID      = '12345678-1234-5678-1234-56789abcdef3'

STATUS_INTERVAL_S = 1.0


# ---------------------------------------------------------------------------
# BT Node
# ---------------------------------------------------------------------------

class BtNode(Node):

    def __init__(self):
        super().__init__('bt_node')

        self.declare_parameter('device_name', 'GLADoS')
        self.declare_parameter('status_interval', STATUS_INTERVAL_S)

        self._device_name      = self.get_parameter('device_name').value
        self._status_interval  = self.get_parameter('status_interval').value

        # Publishers
        self._servo_pub = self.create_publisher(ServoCmd, '/servo_cmd', 10)
        self._rgb_pub   = self.create_publisher(RgbLed,   '/rgb_led',  10)

        # Mirror of last-known servo positions (for STATUS notify)
        self._servo_positions = [90.0, 90.0, 90.0, 90.0]

        # BLE server (runs in its own asyncio thread)
        self._server: Optional[Any] = None
        self._ble_thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        if BLE_AVAILABLE:
            self._start_ble()
        else:
            self.get_logger().warn(
                'bless not installed — BLE unavailable. '
                'Install with: pip install bleak bless')

        # Status notify timer
        self.create_timer(self._status_interval, self._publish_status)

        self.get_logger().info(
            f'bt_node ready — BLE name: "{self._device_name}"')

    # ------------------------------------------------------------------
    # BLE lifecycle
    # ------------------------------------------------------------------

    def _start_ble(self):
        """Spin up the asyncio event loop + GATT server in a daemon thread."""
        self._loop = asyncio.new_event_loop()
        self._ble_thread = threading.Thread(
            target=self._ble_thread_main, daemon=True, name='ble_thread')
        self._ble_thread.start()

    def _ble_thread_main(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._run_gatt_server())

    async def _run_gatt_server(self):
        self._server = BlessServer(name=self._device_name, loop=self._loop)
        self._server.read_request_func  = self._handle_read
        self._server.write_request_func = self._handle_write

        await self._server.add_new_service(SERVICE_UUID)

        # SERVO_CMD — write only
        await self._server.add_new_characteristic(
            SERVICE_UUID, SERVO_CMD_UUID,
            GATTCharacteristicProperties.write |
            GATTCharacteristicProperties.write_without_response,
            None,
            GATTAttributePermissions.writeable,
        )

        # RGB_LED — write only
        await self._server.add_new_characteristic(
            SERVICE_UUID, RGB_LED_UUID,
            GATTCharacteristicProperties.write |
            GATTCharacteristicProperties.write_without_response,
            None,
            GATTAttributePermissions.writeable,
        )

        # STATUS — read + notify
        await self._server.add_new_characteristic(
            SERVICE_UUID, STATUS_UUID,
            GATTCharacteristicProperties.read |
            GATTCharacteristicProperties.notify,
            None,
            GATTAttributePermissions.readable,
        )

        await self._server.start()
        self.get_logger().info('BLE GATT server advertising…')

        # Keep the loop alive
        while rclpy.ok():
            await asyncio.sleep(1.0)

        await self._server.stop()

    # ------------------------------------------------------------------
    # GATT callbacks (called from asyncio thread — use thread-safe publish)
    # ------------------------------------------------------------------

    def _handle_read(self, characteristic: 'BlessGATTCharacteristic',
                     **kwargs) -> bytearray:
        if characteristic.uuid.lower() == STATUS_UUID.lower():
            payload = self._make_status_payload()
            return bytearray(payload.encode())
        return bytearray()

    def _handle_write(self, characteristic: 'BlessGATTCharacteristic',
                      value: Any, **kwargs):
        uuid = characteristic.uuid.lower()
        try:
            raw = bytes(value).decode('utf-8')
            data = json.loads(raw)
        except Exception as e:
            self.get_logger().warn(f'BLE write parse error: {e}  raw={value!r}')
            return

        if uuid == SERVO_CMD_UUID.lower():
            self._handle_servo_cmd(data)
        elif uuid == RGB_LED_UUID.lower():
            self._handle_rgb_cmd(data)

    # ------------------------------------------------------------------
    # Message builders + ROS publishers
    # ------------------------------------------------------------------

    def _handle_servo_cmd(self, data: dict):
        angles = data.get('angles', [-1.0] * 4)
        speeds = data.get('speeds', [1.0] * 4)

        if len(angles) != 4 or len(speeds) != 4:
            self.get_logger().warn('servo_cmd: expected 4 angles and 4 speeds')
            return

        msg = ServoCmd()
        msg.angles = [float(a) for a in angles]
        msg.speeds = [float(s) for s in speeds]

        # Mirror positions for status (skip -1 = no-change entries)
        for i, a in enumerate(msg.angles):
            if a >= 0.0:
                self._servo_positions[i] = a

        self._servo_pub.publish(msg)
        self.get_logger().debug(f'Published /servo_cmd: angles={msg.angles}')

    def _handle_rgb_cmd(self, data: dict):
        msg = RgbLed()
        msg.r = int(max(0, min(255, data.get('r', 0))))
        msg.g = int(max(0, min(255, data.get('g', 0))))
        msg.b = int(max(0, min(255, data.get('b', 0))))
        self._rgb_pub.publish(msg)
        self.get_logger().debug(
            f'Published /rgb_led: R={msg.r} G={msg.g} B={msg.b}')

    # ------------------------------------------------------------------
    # Status notify
    # ------------------------------------------------------------------

    def _make_status_payload(self) -> str:
        return json.dumps({
            'connected': True,
            'servos': [float(v) for v in self._servo_positions],
        })

    def _publish_status(self):
        """Timer callback — push status to BLE notify characteristic."""
        if not (self._server and BLE_AVAILABLE):
            return
        if not self._loop or not self._loop.is_running():
            return
        payload = self._make_status_payload().encode()
        try:
            asyncio.run_coroutine_threadsafe(
                self._notify_status(payload), self._loop)
        except Exception as e:
            self.get_logger().debug(f'Status schedule error: {e}')

    async def _notify_status(self, payload: bytes):
        try:
            char = self._server.get_characteristic(STATUS_UUID)
            if char is None:
                return
            char.value = bytearray(payload)
            await self._server.update_value(SERVICE_UUID, STATUS_UUID)
        except Exception as e:
            self.get_logger().debug(f'Status notify error: {e}')

    def destroy_node(self):
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BtNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
