import argparse
from pathlib import Path

import yaml

registry = yaml.safe_load(Path(__file__).with_name('registry.yaml').read_text())
parser = argparse.ArgumentParser()
commands = parser.add_subparsers(dest='command', required=True)
commands.add_parser('list')
run = commands.add_parser('run')
run.add_argument('name', choices=registry)
run.add_argument('--env', default='isaac')
args = parser.parse_args()
if args.command == 'list':
    for name, spec in registry.items():
        print(name, spec['kind'], ','.join(spec['environments']), spec['description'], sep=' | ')
    raise SystemExit(0)
if args.env not in registry[args.name]['environments'] or args.env != 'isaac':
    parser.error('Unsupported execution environment')

import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from piper_x_interfaces.action import RunTask

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('piper_task_launcher')
client = ActionClient(node, RunTask, '/piper_x/run_task')
try:
    if not client.wait_for_server(timeout_sec=10):
        raise SystemExit('Driver task service unavailable')
    future = client.send_goal_async(RunTask.Goal(name=args.name), feedback_callback=lambda msg:
        print(f'{args.name} | {msg.feedback.run_id} | {msg.feedback.phase}', flush=True))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done() or not future.result().accepted:
        raise SystemExit('Task acceptance not confirmed')
    handle = future.result()
    result = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result)
    except KeyboardInterrupt:
        rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=5)
        rclpy.spin_until_future_complete(node, result, timeout_sec=20)
    if not result.done():
        raise SystemExit('Task terminal state unconfirmed')
    reply = result.result().result
    print('FAIL' if reply.result.error_code else 'PASS', reply.result.message, reply.log_path)
    raise SystemExit(int(bool(reply.result.error_code)))
finally:
    node.destroy_node()
    rclpy.try_shutdown()
