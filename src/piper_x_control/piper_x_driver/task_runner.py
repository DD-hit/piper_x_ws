import base64
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import yaml
from rclpy.serialization import deserialize_message, serialize_message
from piper_x_interfaces.action import RunTask
from piper_x_interfaces.msg import MotionResult


class ChildGoal:
    def __init__(self, request, canceled):
        self.request = request
        self.canceled_event = canceled

    @property
    def is_cancel_requested(self):
        return self.canceled_event.is_set()

    def publish_feedback(self, feedback):
        pass

    def succeed(self):
        pass

    abort = canceled = succeed


def serve_child(service, connection, canceled, run_id):
    with connection.makefile('rwb', buffering=0) as stream:
        while not canceled.is_set():
            line = stream.readline(65536)
            if not line:
                return
            command = json.loads(line)
            name = command['action']
            kind = service.kinds[name]
            request = deserialize_message(base64.b64decode(command['goal'], validate=True), kind.Goal)
            result = service.execute(ChildGoal(request, canceled), kind, name, run_id)
            stream.write(base64.b64encode(serialize_message(result)) + b'\n')


def run_task(service, handle):
    result = RunTask.Result()
    name = handle.request.name
    driver_root = Path(os.environ.get('PIPER_X_WS_ROOT', Path.cwd()))
    root = Path(os.environ.get('PIPER_TASKS_ROOT', driver_root / 'piper_x_tasks')).resolve()
    try:
        spec = yaml.safe_load((root / 'registry.yaml').read_text())[name]
        entry = (root / spec['entry']).resolve()
        category = {'test': 'tests', 'production': 'production'}[spec['kind']]
        if not entry.is_relative_to(root / category) or not entry.is_file() or os.getenv('PIPER_EXECUTION_ENV', 'isaac') not in spec['environments']:
            raise ValueError('Task entry or environment unavailable')
    except (KeyError, TypeError, ValueError, OSError, yaml.YAMLError) as exc:
        result.result = MotionResult(error_code=MotionResult.INVALID_GOAL, message=f'Invalid registered task: {exc}')
        handle.abort()
        return result
    run_id = uuid.uuid4().hex
    with service.lock:
        code = (MotionResult.BUSY if service.owner or service.active else MotionResult.FAULT if service.marker.exists()
                else MotionResult.NOT_READY if time.monotonic() - service.motion.received > 2 else MotionResult.OK)
        if code:
            result.result = MotionResult(error_code=code, message='Driver busy or unavailable')
            handle.abort()
            return result
        service.owner = name
        service.run_id = run_id
    result.run_id = run_id
    process = worker = parent = child = None
    canceled = threading.Event()
    errors = []
    try:
        path = entry.parent / 'logs' / f'{run_id}.log'
        path.parent.mkdir(exist_ok=True)
        result.log_path = str(path)
        parent, child = socket.socketpair()
        env = dict(os.environ, PIPER_TASK_FD=str(child.fileno()), PIPER_TASK_NAME=name,
                   PIPER_TASK_ENV='isaac', PYTHONUNBUFFERED='1', PIPER_RUN_ID=run_id)
        with path.open('x') as log:
            print(f'{name} {run_id}', file=log, flush=True)
            process = subprocess.Popen([sys.executable, str(entry)], cwd=entry.parent, env=env,
                pass_fds=(child.fileno(),), start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
            child.close()

            def serve():
                try:
                    serve_child(service, parent, canceled, run_id)
                except Exception as exc:
                    errors.append(str(exc))
                    canceled.set()

            worker = threading.Thread(target=serve, daemon=True)
            worker.start()
            service.get_logger().info(f'INFO: task={name} run={run_id} pid={process.pid} started')
            handle.publish_feedback(RunTask.Feedback(run_id=run_id, phase='running'))
            while process.poll() is None and not handle.is_cancel_requested and not canceled.wait(0.05):
                pass
            if handle.is_cancel_requested:
                raise InterruptedError('Task canceled')
            if errors or process.returncode != 0:
                raise RuntimeError(f'Child exit={process.returncode}: {errors}; see {path}')
    except Exception as exc:
        cause = service.last if service.last.error_code and service.last_operation_run == run_id else None
        code = (MotionResult.CANCELED if isinstance(exc, InterruptedError) else
                cause.error_code if cause else MotionResult.EXECUTION_FAILED)
        message = f'{exc}; {cause.message}' if cause else str(exc)
        result.result = MotionResult(error_code=code, message=message)
    finally:
        canceled.set()
        if process:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if parent:
            try:
                parent.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if worker:
            worker.join(15)
        if (worker and worker.is_alive()) or service.motion.uncertain:
            service.motion.latch('Task stopped; motion terminal state unconfirmed')
            result.result = MotionResult(error_code=MotionResult.STOP_UNCONFIRMED, message='Stop unconfirmed')
        for channel in (parent, child):
            if channel:
                channel.close()
        with service.lock:
            service.owner = service.run_id = ''
            service.last = result.result
        service.get_logger().info(f'INFO: task={name} run={run_id} result={result.result.error_code}')
    if result.result.error_code == MotionResult.CANCELED and handle.is_cancel_requested:
        handle.canceled()
    elif result.result.error_code:
        handle.abort()
    else:
        handle.succeed()
    return result
