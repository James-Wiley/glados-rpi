#!/usr/bin/env python3
"""
sound_node.py
=============
Plays audio feedback at two points in the robot startup sequence:

  1. INIT  — as soon as sound_node itself starts (robot is booting)
  2. READY — once all expected nodes are alive on the ROS2 graph

Uses `aplay` (ALSA) to play WAV files through a USB audio dongle.
No extra Python packages required.

Audio file locations (configurable via ROS params):
  /home/fishbot/sounds/init.wav
  /home/fishbot/sounds/ready.wav

ROS params:
  sound_init        (str)   path to init WAV file
  sound_ready       (str)   path to ready WAV file
  ready_nodes       (str)   comma-separated list of node names to wait for
  check_interval    (float) seconds between node-list polls (default 2.0)
  aplay_device      (str)   ALSA device string, e.g. 'hw:1,0' (default 'default')
"""

import os
import subprocess
import threading

import rclpy
from rclpy.node import Node


DEFAULT_READY_NODES = '/servo_node,/rgb_led_node,/bt_node'


class SoundNode(Node):

    def __init__(self):
        super().__init__('sound_node')

        # --- Parameters -------------------------------------------------------
        self.declare_parameter('sound_init',     '/home/fishbot/sounds/init.wav')
        self.declare_parameter('sound_ready',    '/home/fishbot/sounds/ready.wav')
        self.declare_parameter('ready_nodes',    DEFAULT_READY_NODES)
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('aplay_device',   'default')

        self._init_file    = self.get_parameter('sound_init').value
        self._ready_file   = self.get_parameter('sound_ready').value
        self._ready_nodes  = [
            n.strip() for n in
            self.get_parameter('ready_nodes').value.split(',')
        ]
        self._check_interval = self.get_parameter('check_interval').value
        self._device         = self.get_parameter('aplay_device').value

        self._ready_played = False

        # Play init sound immediately in a thread so we don't block spinning
        threading.Thread(target=self._play, args=(self._init_file,),
                         daemon=True).start()
        self.get_logger().info('Playing init sound')

        # Poll the node graph until all expected nodes are up
        self._timer = self.create_timer(self._check_interval, self._check_nodes)

    # ------------------------------------------------------------------

    def _play(self, path: str):
        """Play a WAV file via aplay (blocking, runs in its own thread)."""
        if not os.path.isfile(path):
            self.get_logger().warn(
                f'Audio file not found: {path}\n'
                f'Place your WAV file there or update the sound_init / '
                f'sound_ready ROS params.')
            return
        try:
            subprocess.run(
                ['aplay', '-D', self._device, path],
                check=True,
                capture_output=True,
            )
        except subprocess.CalledProcessError as e:
            self.get_logger().error(
                f'aplay failed for {path}: {e.stderr.decode().strip()}')
        except FileNotFoundError:
            self.get_logger().error(
                'aplay not found — install with: sudo apt install alsa-utils')

    def _check_nodes(self):
        """Timer callback — fires every check_interval seconds."""
        if self._ready_played:
            self._timer.cancel()
            return

        active = self.get_node_names_and_namespaces()
        # get_node_names_and_namespaces returns list of (name, namespace) tuples
        active_full = [
            (ns.rstrip('/') + '/' + name).replace('//', '/')
            for name, ns in active
        ]

        missing = [n for n in self._ready_nodes if n not in active_full]

        if missing:
            self.get_logger().debug(f'Waiting for nodes: {missing}')
        else:
            self._ready_played = True
            self._timer.cancel()
            self.get_logger().info('All nodes ready — playing ready sound')
            threading.Thread(target=self._play, args=(self._ready_file,),
                             daemon=True).start()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
