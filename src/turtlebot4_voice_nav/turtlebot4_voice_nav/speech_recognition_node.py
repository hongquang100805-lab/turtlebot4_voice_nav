"""Explicitly armed local Vietnamese speech recognition, with no robot commands."""

import json
import math
from pathlib import Path
import queue
import shutil
import subprocess
import tempfile
import threading
import time
import wave

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SpeechRecognitionNode(Node):
    """One /voice/listen call captures one utterance; no background command queue."""

    def __init__(self):
        super().__init__('speech_recognition_node')
        defaults = {
            'model_path': str(Path.home() / 'tb4_ws/models/whisper-small'),
            'audio_device': 'default', 'record_seconds': 6.0,
            'cpu_threads': 4, 'min_rms': 0.006, 'min_voiced_seconds': 0.30,
            'min_avg_logprob': -0.75, 'max_no_speech_prob': 0.5,
            'wav_file': '',
            'initial_prompt': ('Đi đến khu vực một. Đi đến khu vực hai. Đi đến khu vực ba. '
                               'Di chuyển từ khu vực một đến khu vực hai. '
                               'Đi đến nhà ăn. Đi đến khu vực Z. Hủy điều hướng.'),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        for name in ('record_seconds', 'min_rms', 'min_voiced_seconds',
                     'min_avg_logprob', 'max_no_speech_prob'):
            if not math.isfinite(self.settings[name]):
                raise ValueError(f'{name} must be finite')
        if not 1 <= self.settings['record_seconds'] <= 20:
            raise ValueError('record_seconds must be between 1 and 20 seconds')
        if not 1 <= self.settings['cpu_threads'] <= 16:
            raise ValueError('cpu_threads must be between 1 and 16')
        if not 0 < self.settings['min_rms'] < 1 or self.settings['min_voiced_seconds'] <= 0:
            raise ValueError('Invalid audio energy/duration thresholds')
        if not 0 <= self.settings['max_no_speech_prob'] <= 1:
            raise ValueError('max_no_speech_prob must be between zero and one')
        self.transcripts = self.create_publisher(String, '/voice/transcript', 1)
        self.statuses = self.create_publisher(String, '/voice/asr_status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.requests = queue.Queue(maxsize=1)
        self.events = queue.Queue(maxsize=12)
        self.lock = threading.Lock()
        self.ready = self.busy = False
        self.stop_event = threading.Event()
        self.service = self.create_service(Trigger, '/voice/listen', self.listen)
        self.timer = self.create_timer(0.05, self.drain,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.worker = threading.Thread(target=self.run_worker, daemon=True)
        self.worker.start()

    def event(self, state, message, **data):
        packet = {'state': state, 'message': message, **data}
        try:
            self.events.put_nowait(packet)
        except queue.Full:
            self.events.get_nowait()
            self.events.put_nowait(packet)

    def drain(self):
        while not self.events.empty():
            packet = self.events.get_nowait()
            self.statuses.publish(String(data=json.dumps(packet, ensure_ascii=False)))
            self.get_logger().info(json.dumps(packet, ensure_ascii=False))
            if packet['state'] == 'TRANSCRIBED':
                self.transcripts.publish(String(data=packet['text']))

    def listen(self, request, response):
        del request
        with self.lock:
            if not self.ready:
                response.success = False
                response.message = 'ASR chưa sẵn sàng; xem /voice/asr_status'
            elif self.busy:
                response.success = False
                response.message = 'Đang thu/nhận dạng; không xếp hàng câu lệnh cũ'
            else:
                self.busy = True
                self.requests.put_nowait(None)
                response.success = True
                response.message = f'Đã nhận yêu cầu: nói một câu trong {self.settings["record_seconds"]:g} giây'
        return response

    def capture(self, filename):
        if not shutil.which('arecord'):
            raise RuntimeError('Thiếu arecord; cài alsa-utils hoặc truyền wav_file để kiểm tra ASR')
        self.event('RECORDING', 'Nói câu lệnh ngay bây giờ',
                   seconds=self.settings['record_seconds'], device=self.settings['audio_device'])
        command = ['arecord', '-q', '-D', self.settings['audio_device'],
                   '-t', 'wav', '-f', 'S16_LE', '-r', '16000', '-c', '1',
                   '-d', str(math.ceil(self.settings['record_seconds'])), str(filename)]
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        deadline = time.monotonic() + self.settings['record_seconds'] + 6
        try:
            while process.poll() is None:
                if self.stop_event.wait(0.05) or time.monotonic() > deadline:
                    process.terminate()
                    raise RuntimeError('Đã dừng thu âm hoặc hết thời gian chờ microphone')
            error = process.communicate(timeout=2)[1].decode(errors='replace').strip()
            if process.returncode:
                raise RuntimeError(f'Không thu được microphone: {error}')
        finally:
            if process.poll() is None:
                process.kill()
            process.communicate()

    def transcribe(self, model, filename):
        with wave.open(str(filename), 'rb') as audio:
            if (audio.getnchannels(), audio.getsampwidth(), audio.getframerate()) != (1, 2, 16000):
                raise ValueError('WAV phải là PCM signed 16-bit, mono, 16000 Hz')
            samples = np.frombuffer(audio.readframes(audio.getnframes()), dtype='<i2')
        if len(samples) < 1600 or len(samples) > 16000 * 30:
            raise ValueError('Độ dài WAV phải từ 0,1 đến 30 giây')
        samples = samples.astype(np.float32) / 32768.0
        rms = float(np.sqrt(np.mean(samples ** 2)))
        frame_count = len(samples) // 320
        levels = np.sqrt(np.mean(samples[:frame_count * 320].reshape(-1, 320) ** 2, axis=1))
        voiced = float(np.count_nonzero(levels >= self.settings['min_rms']) * 0.02)
        if rms < self.settings['min_rms'] or voiced < self.settings['min_voiced_seconds']:
            self.event('REJECTED', 'Âm quá nhỏ/im lặng: không publish câu lệnh',
                       rms=round(rms, 5), voiced_seconds=round(voiced, 2))
            return
        self.event('TRANSCRIBING', 'Đang nhận dạng tiếng Việt trên CPU')
        start = time.monotonic()
        segments, info = model.transcribe(
            samples, language='vi', beam_size=5, vad_filter=True,
            condition_on_previous_text=False, temperature=0.0,
            initial_prompt=self.settings['initial_prompt'] or None,
            no_speech_threshold=self.settings['max_no_speech_prob'],
            log_prob_threshold=self.settings['min_avg_logprob'],
        )
        segments = list(segments)
        if not segments:
            self.event('REJECTED', 'Không có lời nói đủ tin cậy')
            return
        # Reject the entire utterance instead of silently dropping an uncertain stop.
        if any(s.avg_logprob < self.settings['min_avg_logprob'] or
               s.no_speech_prob > self.settings['max_no_speech_prob'] for s in segments):
            self.event('REJECTED', 'Nhận dạng chưa đủ tin cậy; hãy nói lại',
                       candidate_text=' '.join(s.text.strip() for s in segments),
                       segment_scores=[{'avg_logprob': round(s.avg_logprob, 3),
                                        'no_speech_prob': round(s.no_speech_prob, 3)}
                                       for s in segments])
            return
        text = ' '.join(s.text.strip() for s in segments).strip()
        if not text:
            self.event('REJECTED', 'Văn bản nhận dạng rỗng')
            return
        self.event('TRANSCRIBED', 'Đã nhận dạng; chuyển văn bản sang node điều hướng',
                   text=text, language=info.language, rms=round(rms, 5),
                   inference_seconds=round(time.monotonic() - start, 3),
                   avg_logprob=round(sum(s.avg_logprob for s in segments) / len(segments), 3))

    def run_worker(self):
        self.event('LOADING', 'Đang nạp Whisper từ ổ đĩa, không thu microphone')
        try:
            from faster_whisper import WhisperModel
            model_path = Path(self.settings['model_path']).expanduser()
            if not (model_path / 'model.bin').is_file():
                raise RuntimeError('Chưa có model Whisper; chạy scripts/setup_voice.sh')
            model = WhisperModel(str(model_path), device='cpu', compute_type='int8',
                                 cpu_threads=self.settings['cpu_threads'], local_files_only=True)
            with self.lock:
                self.ready = True
            self.event('READY', 'Gọi /voice/listen để thu một câu; hiện không thu microphone')
            wav_file = self.settings['wav_file']
            if wav_file:
                with self.lock:
                    self.busy = True
                self.requests.put_nowait(str(Path(wav_file).expanduser().resolve()))
            while not self.stop_event.is_set():
                try:
                    source = self.requests.get(timeout=0.2)
                except queue.Empty:
                    continue
                try:
                    with tempfile.TemporaryDirectory(prefix='tb4_voice_') as temp:
                        filename = Path(source) if source else Path(temp) / 'utterance.wav'
                        if source is None:
                            self.capture(filename)
                        if not self.stop_event.is_set():
                            self.transcribe(model, filename)
                except Exception as error:
                    self.event('ERROR', str(error))
                finally:
                    with self.lock:
                        self.busy = False
                    self.event('READY', 'Có thể gọi /voice/listen cho câu tiếp theo')
        except Exception as error:
            with self.lock:
                self.ready = False
            self.event('ERROR', str(error))

    def close(self):
        self.stop_event.set()
        self.worker.join(timeout=2)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeechRecognitionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
