"""Streaming microphone adapter: Silero VAD plus offline SenseVoice ASR."""

from __future__ import annotations

import queue
import time
from typing import Iterator

from .intents import VoiceObservation, infer_observation

SAMPLE_RATE = 16000


def microphone_observations(
    sense_voice_model: str,
    tokens: str,
    silero_vad_model: str,
    *,
    num_threads: int = 2,
    device: int | None = None,
) -> Iterator[VoiceObservation]:
    """Yield final recognised utterances from a local microphone.

    Imports stay inside the function so a UART-only deployment has no machine
    learning dependency.  Speech is segmented before ASR, keeping CPU work
    bounded while the robot is moving and the camera service is active.
    """

    try:
        import numpy as np
        import sherpa_onnx
        import sounddevice as sd
    except ImportError as exc:
        raise RuntimeError(
            "microphone mode needs numpy, sounddevice, and sherpa-onnx; run install_voice_runtime.sh"
        ) from exc

    audio_queue: queue.Queue[object] = queue.Queue()

    def audio_callback(indata: object, _frames: int, _time: object, status: object) -> None:
        if status:
            print(f"[VOICE] microphone status: {status}", flush=True)
        audio_queue.put(np.copy(indata).reshape(-1))

    recognizer = sherpa_onnx.OfflineRecognizer.from_sense_voice(
        model=sense_voice_model,
        tokens=tokens,
        num_threads=max(1, int(num_threads)),
        use_itn=False,
        debug=False,
    )
    config = sherpa_onnx.VadModelConfig()
    config.silero_vad.model = silero_vad_model
    config.silero_vad.threshold = 0.5
    config.silero_vad.min_silence_duration = 0.25
    config.silero_vad.min_speech_duration = 0.30
    config.silero_vad.max_speech_duration = 5.0
    config.sample_rate = SAMPLE_RATE
    vad = sherpa_onnx.VoiceActivityDetector(config, buffer_size_in_seconds=20)
    window_size = config.silero_vad.window_size

    with sd.InputStream(
        channels=1,
        dtype="float32",
        samplerate=SAMPLE_RATE,
        blocksize=int(SAMPLE_RATE * 0.1),
        device=device,
        callback=audio_callback,
    ):
        print("[VOICE] microphone ASR ready", flush=True)
        while True:
            samples = audio_queue.get()
            offset = 0
            while offset + window_size <= len(samples):
                vad.accept_waveform(samples[offset : offset + window_size])
                offset += window_size
            while not vad.empty():
                segment = vad.front.samples
                vad.pop()
                if len(segment) < int(SAMPLE_RATE * 0.30):
                    continue
                stream = recognizer.create_stream()
                stream.accept_waveform(SAMPLE_RATE, segment)
                recognizer.decode_stream(stream)
                transcript = stream.result.text.strip()
                if transcript:
                    yield infer_observation(transcript)
            # Give other Python worker threads a scheduling point on busy Pi CPUs.
            time.sleep(0.001)
