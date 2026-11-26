import json
import os
from pathlib import Path
import signal
import subprocess
import time
from datetime import datetime

root = Path(__file__).resolve().parents[1]
report_dir = root / '.runtime' / ('acceptance-' + datetime.now().strftime('%Y%m%d-%H%M%S'))
report_dir.mkdir()
results = []
try:
    for barrier, tasks in [(False, ['empty_motion', 'cartesian_line', 'cartesian_arc', 'fault_handling']),
                            (True, ['obstacle_avoidance'])]:
        mode = 'barrier' if barrier else 'empty'
        for lock in ('isaac', 'isaac-services', 'isaac-physics'):
            subprocess.run(['flock', '-n', str(root / '.runtime' / (lock + '.lock')), 'true'], check=True)
        log_path = report_dir / (mode + '-driver.log')
        with log_path.open('w') as output:
            process = subprocess.Popen(['bash', 'scripts/run_isaac.sh'], cwd=root,
                env=dict(os.environ, PIPER_DEMO_BARRIER=str(int(barrier))), start_new_session=True,
                stdout=output, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 150
                while 'ROS 2 task actions ready' not in log_path.read_text():
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('Driver startup failed: ' + str(log_path))
                    time.sleep(0.5)
                for task in tasks:
                    start = time.monotonic()
                    with (report_dir / (task + '.log')).open('w') as log:
                        response = subprocess.run(['bash', 'run_isaac_task.sh', 'run', 'test.' + task],
                            cwd=root / 'piper_x_tasks', stdout=log, stderr=subprocess.STDOUT)
                    results.append(dict(task=task, passed=response.returncode == 0,
                                        elapsed_seconds=round(time.monotonic() - start, 3)))
            finally:
                os.kill(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                    raise RuntimeError('Driver cleanup timeout; check child processes')
finally:
    (report_dir / 'report.json').write_text(json.dumps(results, indent=2))
    print(report_dir, flush=True)
raise SystemExit(int(not results or not all(result['passed'] for result in results)))
