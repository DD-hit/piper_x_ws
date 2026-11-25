"""Read installed description data without importing ROS modules in Isaac Python."""
import json
import os
from pathlib import Path

share = os.environ.get('PIPER_DESCRIPTION_SHARE')
root = Path(share) if share else Path(__file__).resolve().parents[2] / 'piper_x_description'
CONFIG = json.loads((root / 'config/gemini_frames.json').read_text())
