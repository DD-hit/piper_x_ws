import json
import os
import socket


class Arm:
    def __enter__(self):
        self.socket = socket.socket(socket.AF_UNIX)
        self.socket.settimeout(75)
        try:
            self.socket.connect(os.environ.get('PIPER_TASK_SOCKET', f'/tmp/piper-x-{os.getuid()}-isaac.sock'))
            self.stream = self.socket.makefile('rb')
            hello = self.receive()
            if hello.get('backend') != os.environ['PIPER_TASK_ENV']:
                raise RuntimeError('运行环境与注册项不匹配')
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def receive(self):
        line = self.stream.readline()
        if not line:
            raise ConnectionError('driver 连接中断，检查服务日志及运动状态')
        response = json.loads(line)
        if 'error' in response:
            raise RuntimeError(response['error'])
        return response

    def call(self, operation, *args):
        print(f'{operation} {args}', flush=True)
        self.socket.sendall(json.dumps({'op': operation, 'args': args}, allow_nan=False).encode() + b'\n')
        return self.receive()['result']

    def __exit__(self, *exc):
        if hasattr(self, 'stream'):
            self.stream.close()
        self.socket.close()
