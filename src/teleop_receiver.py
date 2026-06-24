#!/usr/bin/env python3
"""
分布式机械臂主从遥操作 TCP 接收节点。

运行在 APEX AD10 侧，连接本地电脑发送端，解析 44 字节的
关节目标二进制帧，并重新发布为 /slave/joint_states。
"""

import socket
import struct
import threading
import time
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState


FRAME_MAGIC = 0x4D41524D
FRAME_FORMAT = '<IIiI6fI'
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)
DEFAULT_JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6_left',
]


def unpack_frame(raw):
    """解析并校验单个 44 字节关节目标帧。"""
    if len(raw) != FRAME_SIZE:
        return None

    unpacked = struct.unpack(FRAME_FORMAT, raw)
    magic = unpacked[0]
    crc = unpacked[-1]

    if magic != FRAME_MAGIC:
        return None

    expected_crc = zlib.crc32(raw[:40]) & 0xFFFFFFFF
    if crc != expected_crc:
        return None

    return {
        'seq': unpacked[1],
        'ts_sec': unpacked[2],
        'ts_nsec': unpacked[3],
        'positions': list(unpacked[4:10]),
    }


class TeleopReceiver(Node):
    """接收远端从臂目标关节数据，并发布到 /slave/joint_states。"""

    def __init__(self):
        super().__init__('teleop_receiver')

        self.declare_parameter('server_host', '192.168.1.100')
        self.declare_parameter('server_port', 5005)
        self.declare_parameter('reconnect_base_delay', 1.0)
        self.declare_parameter('reconnect_max_delay', 30.0)
        self.declare_parameter('publish_topic', '/slave/joint_states')
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)

        self._host = str(self.get_parameter('server_host').value)
        self._port = int(self.get_parameter('server_port').value)
        self._base_delay = float(self.get_parameter('reconnect_base_delay').value)
        self._max_delay = float(self.get_parameter('reconnect_max_delay').value)
        self._publish_topic = str(self.get_parameter('publish_topic').value)
        self._joint_names = list(self.get_parameter('joint_names').value)
        if len(self._joint_names) != 6:
            self.get_logger().warning(
                'joint_names length is not 6, falling back to default names.'
            )
            self._joint_names = list(DEFAULT_JOINT_NAMES)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            JointState, self._publish_topic, qos)

        self._running = True
        self._last_seq = None
        self._recv_thread = threading.Thread(
            target=self._reconnect_loop,
            daemon=True,
            name='teleop-recv',
        )
        self._recv_thread.start()

        self._publish_zero()
        self.get_logger().info(
            f'TeleopReceiver ready, will connect to {self._host}:{self._port} '
            f'and publish {self._publish_topic}.'
        )

    def destroy_node(self):
        self._running = False
        if self._recv_thread.is_alive():
            self._recv_thread.join(timeout=2.0)
        super().destroy_node()

    def _publish_zero(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = list(self._joint_names)
        msg.position = [0.0] * len(self._joint_names)
        msg.velocity = [0.0] * len(self._joint_names)
        msg.effort = [0.0] * len(self._joint_names)
        self._publisher.publish(msg)

    def _reconnect_loop(self):
        delay = self._base_delay
        sock = None

        while self._running:
            if sock is None:
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(1.0)
                    sock.connect((self._host, self._port))
                    sock.settimeout(None)
                    self._last_seq = None
                    delay = self._base_delay
                    self.get_logger().info(
                        f'Connected to teleop sender at {self._host}:{self._port}')
                except (ConnectionRefusedError, socket.timeout, OSError) as exc:
                    self.get_logger().debug(
                        f'Connect failed ({exc}), retry in {delay:.1f}s')
                    if sock is not None:
                        sock.close()
                        sock = None
                    time.sleep(delay)
                    delay = min(delay * 2.0, self._max_delay)
                    continue

            try:
                raw = self._recv_exact(sock, FRAME_SIZE)
                if raw is None:
                    self.get_logger().warning('Teleop sender closed connection.')
                    sock.close()
                    sock = None
                    continue

                self._handle_frame(raw)
            except (ConnectionResetError, BrokenPipeError, OSError) as exc:
                self.get_logger().warning(f'Connection lost: {exc}')
                try:
                    sock.close()
                except OSError:
                    pass
                sock = None

    def _recv_exact(self, sock, size):
        data = bytearray()
        while len(data) < size and self._running:
            chunk = sock.recv(size - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    def _handle_frame(self, raw):
        parsed = unpack_frame(raw)
        if parsed is None:
            self.get_logger().warning('Corrupt teleop frame received.')
            return

        if self._last_seq is not None:
            gap = parsed['seq'] - self._last_seq - 1
            if gap > 0:
                self.get_logger().warning(f'Sequence gap: {gap} frame(s) lost.')
            elif gap < 0:
                self.get_logger().debug(
                    f'Sequence reset (was {self._last_seq}, now {parsed["seq"]}).')
        self._last_seq = parsed['seq']

        msg = JointState()
        msg.header.stamp.sec = parsed['ts_sec']
        msg.header.stamp.nanosec = parsed['ts_nsec']
        msg.header.frame_id = ''
        msg.name = list(self._joint_names)
        msg.position = list(parsed['positions'])
        msg.velocity = [0.0] * len(msg.position)
        msg.effort = [0.0] * len(msg.position)
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
