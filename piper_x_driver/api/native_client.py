import base64
import json
import os
import socket

import rclpy
from rclpy.serialization import deserialize_message, serialize_message


class Arm:
    def __enter__(self):
        self.channel = socket.socket(fileno=os.dup(int(os.environ['PIPER_TASK_FD'])))
        self.stream = self.channel.makefile('rwb', buffering=0)
        rclpy.init()
        self.node = rclpy.create_node('piper_x_test_client')
        return self

    def wait(self, future, seconds):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=seconds)
        if not future.done():
            raise TimeoutError('ROS response timed out')
        return future.result()

    def run(self, kind, name, **fields):
        goal = base64.b64encode(serialize_message(kind.Goal(**fields))).decode()
        self.stream.write(json.dumps(dict(action=name, goal=goal)).encode() + b'\n')
        import select
        while not select.select([self.channel], [], [], 0)[0]:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        line = self.stream.readline(65536)
        if not line:
            raise ConnectionError('Driver disconnected')
        result = deserialize_message(base64.b64decode(line.rstrip(b'\n'), validate=True), kind.Result)
        if result.result.error_code:
            raise RuntimeError(f'{name}: {result.result}')
        print(f'{name}: {result}', flush=True)
        return result

    def __exit__(self, *exc):
        self.stream.close()
        self.channel.close()
        self.node.destroy_node()
        rclpy.try_shutdown()
