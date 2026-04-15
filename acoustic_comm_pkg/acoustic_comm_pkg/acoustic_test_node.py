import threading
import time

import serial

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn


class AcousticSerialAssistantNode(LifecycleNode):
    def __init__(self):
        super().__init__('acoustic_serial_assistant_node')

        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 19200)

        self.serial_port = None

        self.node_active = False
        self.is_running = False
        self.input_thread = None
        self.rx_thread = None

        self.get_logger().info('acoustic_serial_assistant_node created.')

    def on_configure(self, state):
        self.get_logger().info('Configuring node...')

        try:
            self.port_name = self.get_parameter('port_name').value
            self.baudrate = self.get_parameter('baudrate').value

            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=0.1
            )

            self.get_logger().info(
                f'Serial opened: {self.port_name}, baudrate={self.baudrate}'
            )
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('Activating node...')

        if self.serial_port is None:
            self.get_logger().error('Serial port is not available.')
            return TransitionCallbackReturn.FAILURE

        self.node_active = True
        self.is_running = True

        self.input_thread = threading.Thread(
            target=self.keyboard_input_thread,
            daemon=True
        )
        self.input_thread.start()

        self.rx_thread = threading.Thread(
            target=self.serial_receive_thread,
            daemon=True
        )
        self.rx_thread.start()

        self.get_logger().info(
            'Node activated. Please input HEX bytes, for example:\n'
            '24 24 10 02 00 05 01 02 03 04 05 16 40 40'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating node...')

        self.node_active = False
        self.is_running = False

        if self.input_thread is not None and self.input_thread.is_alive():
            self.input_thread.join(timeout=1.0)
            self.input_thread = None

        if self.rx_thread is not None and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
            self.rx_thread = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up node...')

        self.node_active = False
        self.is_running = False

        if self.input_thread is not None and self.input_thread.is_alive():
            self.input_thread.join(timeout=1.0)
            self.input_thread = None

        if self.rx_thread is not None and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
            self.rx_thread = None

        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('Shutting down node...')

        self.node_active = False
        self.is_running = False

        if self.input_thread is not None and self.input_thread.is_alive():
            self.input_thread.join(timeout=1.0)
            self.input_thread = None

        if self.rx_thread is not None and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
            self.rx_thread = None

        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None

        return TransitionCallbackReturn.SUCCESS

    def keyboard_input_thread(self):
        while self.is_running:
            try:
                user_input = input('Please input HEX data to send: ').strip()

                if not self.is_running:
                    break

                if user_input == '':
                    continue

                if self.serial_port is None:
                    print('[ERROR] Serial port is not open.')
                    continue

                # 支持两种输入风格：
                # 1. 24 24 10 02 00 05 ...
                # 2. 242410020005...
                hex_str = user_input.replace(' ', '').replace('\t', '')

                # 十六进制字符串长度必须为偶数
                if len(hex_str) % 2 != 0:
                    print('[ERROR] HEX string length must be even.')
                    continue

                tx_bytes = bytes.fromhex(hex_str)

                self.serial_port.write(tx_bytes)

                print(f'[SEND HEX] {tx_bytes.hex(" ")}')

            except ValueError:
                print('[ERROR] Invalid HEX input. Example:')
            except EOFError:
                break
            except Exception as e:
                print(f'[INPUT ERROR] {e}')
                time.sleep(0.1)

    def serial_receive_thread(self):
        while self.is_running:
            try:
                if self.serial_port is None:
                    time.sleep(0.1)
                    continue

                data = self.serial_port.read(256)

                if data:
                    print(f'[RECV HEX] {data.hex(" ")}')

            except Exception as e:
                print(f'[SERIAL ERROR] {e}')
                time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    node = AcousticSerialAssistantNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
