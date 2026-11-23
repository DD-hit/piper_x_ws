import json
import os
from pathlib import Path
import select
import socket
import socketserver
import signal
import threading
import time
import uuid

import rclpy
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from motion import Motion


class Handler(socketserver.StreamRequestHandler):
    def reply(self, **data):
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode() + b'\n')

    def handle(self):
        server = self.server
        if not server.owner.acquire(blocking=False):
            server.motion.get_logger().warning('BUSY: 拒绝新连接，已有任务持有控制权')
            self.reply(error='BUSY: 已有任务持有控制权')
            return
        motion = server.motion
        operation = '连接'
        run_id = uuid.uuid4().hex[:8]
        action_id = 0
        motion.get_logger().info(f'运行={run_id} 已获取控制权')

        def check_connection():
            if select.select([self.connection], [], [], 0)[0] and not self.connection.recv(1, socket.MSG_PEEK):
                raise ConnectionError('任务已断开')

        try:
            self.connection.settimeout(60)
            motion.check_owner = check_connection
            self.reply(backend='isaac')
            while line := self.rfile.readline(8192):
                request = json.loads(line)
                operation = request['op']
                args = request.get('args', [])
                action_id += 1
                started = time.monotonic()
                motion.get_logger().info(f'运行={run_id} 动作={action_id} 开始 {operation} 目标={args}')
                if server.marker.exists():
                    raise RuntimeError(f'FAULT: {server.marker.read_text()}')
                operations = {'joints': motion.joint_pose, 'shift_tcp_z': motion.shift_tcp,
                              'opening': motion.opening, 'collision': motion.collision}
                function = operations[operation]
                motion.ready()
                motion_started = operation != 'collision'
                if motion_started:
                    server.marker.write_text(f'上次 {operation} 的结束状态待确认')
                try:
                    result = function(*args)
                finally:
                    if motion_started and not server.uncertain:
                        server.marker.unlink(missing_ok=True)
                self.reply(ok=True, result=result)
                motion.get_logger().info(f'运行={run_id} 动作={action_id} 成功，耗时={time.monotonic()-started:.2f} s')
        except Exception as exc:
            motion.get_logger().error(f'运行={run_id} 动作={action_id} {operation} 终止: {exc!r}')
            try:
                self.reply(error=f'{operation}: {exc}')
            except OSError:
                pass
        finally:
            motion.check_owner = lambda: None
            server.owner.release()
            motion.get_logger().info(f'运行={run_id} 控制权已释放')


class Service(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


if __name__ == '__main__':
    os.umask(0o077)
    endpoint = Path(os.environ.get('PIPER_TASK_SOCKET', f'/tmp/piper-x-{os.getuid()}-isaac.sock'))
    endpoint.unlink(missing_ok=True)
    rclpy.init()
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    motion = Motion()
    motion.scene_client = motion.create_client(GetPlanningScene, 'get_planning_scene')
    motion.valid_client = motion.create_client(GetStateValidity, 'check_state_validity')
    motion.get_logger().info('后端=isaac，ROS_DOMAIN_ID=' + os.environ.get('ROS_DOMAIN_ID', '0') +
                             '，正在检查 MoveIt、轨迹控制器、关节反馈和 TF')
    try:
        motion.ready()
    except Exception as exc:
        motion.get_logger().error(f'启动检查失败，任务接口不可用: {exc!r}')
        motion.destroy_node()
        rclpy.try_shutdown()
        raise SystemExit(1)
    with Service(str(endpoint), Handler) as server:
        server.motion = motion
        server.owner = threading.Lock()
        server.uncertain = False
        server.marker = Path('.runtime/task-motion-uncertain')

        def latch(reason):
            server.uncertain = True
            server.marker.write_text(reason)
            motion.get_logger().error(f'FAULT: {reason}')

        motion.latch = latch
        if server.marker.exists():
            motion.get_logger().error(f'FAULT: {server.marker.read_text()}，运动已锁定，检查日志并确认停止状态')
        else:
            motion.get_logger().info(f'MoveIt、控制器、反馈和 TF 检查通过，任务接口就绪 {endpoint}')
        try:
            server.serve_forever()
        finally:
            endpoint.unlink(missing_ok=True)
