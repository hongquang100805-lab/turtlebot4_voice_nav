#!/usr/bin/env bash
# Local Vietnamese ASR dependencies; leaves gesture/YOLO and system Python alone.
set -euo pipefail
TB4_VOICE_WS="${TB4_LAB_WS:-$HOME/tb4_ws}"
TB4_VOICE_ENV="$TB4_VOICE_WS/.venv-voice"
TB4_VOICE_MODEL="${TB4_VOICE_MODEL:-small}"
if [[ ! "$TB4_VOICE_MODEL" =~ ^(tiny|base|small|medium)$ ]]; then
    echo 'TB4_VOICE_MODEL must be tiny, base, small or medium (multilingual, not .en).' >&2
    exit 2
fi
export HF_HOME="$TB4_VOICE_WS/log/huggingface"
export PIP_CACHE_DIR="$TB4_VOICE_WS/log/pip-cache"
mkdir -p "$TB4_VOICE_WS/models" "$PIP_CACHE_DIR"
/usr/bin/python3 -m venv --system-site-packages --without-pip "$TB4_VOICE_ENV"
"$TB4_VOICE_ENV/bin/python" -m pip install \
    'numpy==1.26.4' 'faster-whisper==1.2.1'
"$TB4_VOICE_ENV/bin/python" - "$TB4_VOICE_WS" "$TB4_VOICE_MODEL" <<'PY'
import sys
from pathlib import Path
from faster_whisper import WhisperModel, download_model
import numpy
workspace, size = sys.argv[1:]
output = Path(workspace) / 'models' / ('whisper-' + size)
# Downloads public model files once; runtime node uses local_files_only=True.
download_model(size, output_dir=str(output))
model = WhisperModel(str(output), device='cpu', compute_type='int8',
                     cpu_threads=4, local_files_only=True)
print('Whisper ready:', output, 'NumPy', numpy.__version__)
PY
command -v arecord >/dev/null || echo 'Microphone requires arecord (alsa-utils). WAV diagnostic remains available.'
echo 'Ready: use .venv-voice/bin/python for speech_recognition_node.'
