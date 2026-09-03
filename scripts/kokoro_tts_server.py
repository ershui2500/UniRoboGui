#!/usr/bin/env python3
"""Loopback-only Kokoro zh-en TTS service for UniRoboGui.

Keeps a sherpa-onnx Kokoro INT8 model warm and synthesizes mixed Chinese/English
speech into raw 16 kHz mono s16le PCM. The C++ web service fetches that PCM from
POST /tts and sends it to the robot speaker through Unitree AudioClient::PlayStream.
The service never uses PulseAudio for robot playback.
"""

import argparse
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import sherpa_onnx


_LOW_LATENCY_BREAKS = "，,。！？!?；;\n"
_CONTINUATION_STRONG_BREAKS = "。！？!?；;\n"
_LEAD_SPEECH_UNITS = 6.0


def _split_leading_text(text: str):
    """Split only the first short phrase so speech can start quickly."""
    text = text.strip()
    if not text:
        return []

    speech_units = 0.0
    last_space = -1
    for index, char in enumerate(text):
        if char.isspace():
            last_space = index
            continue

        if (
            char in _LOW_LATENCY_BREAKS
            and speech_units >= _LEAD_SPEECH_UNITS
            and index + 1 < len(text)
            and not (
                char == ","
                and index > 0
                and text[index - 1].isdigit()
                and text[index + 1].isdigit()
            )
        ):
            lead = text[: index + 1].strip()
            rest = text[index + 1 :].strip()
            return [lead, rest] if rest else [lead]

        speech_units += 1.0 if ord(char) > 127 else (0.4 if char.isalnum() else 0.0)
        if speech_units < _LEAD_SPEECH_UNITS or index + 1 >= len(text):
            continue

        cut = index + 1
        if last_space > 0 and last_space >= cut // 2:
            cut = last_space
        lead = text[:cut].rstrip()
        rest = text[cut:].lstrip()
        if lead and rest and lead[-1] not in _LOW_LATENCY_BREAKS:
            lead += "，" if any(ord(c) > 127 for c in lead) else ","
        return [lead, rest] if rest else [lead]

    return [text]


def _split_continuation_text(text: str):
    """Split remaining speech into natural phrases for parallel prefetch."""
    text = text.strip()
    if not text:
        return []

    segments = []
    start = 0
    speech_units = 0.0
    last_space = -1
    for index, char in enumerate(text):
        if char.isspace():
            last_space = index
        else:
            speech_units += 1.0 if ord(char) > 127 else (0.4 if char.isalnum() else 0.0)

        numeric_comma = (
            char == ","
            and index > start
            and index + 1 < len(text)
            and text[index - 1].isdigit()
            and text[index + 1].isdigit()
        )
        strong_break = char in _CONTINUATION_STRONG_BREAKS
        soft_break = char in "，," and not numeric_comma and speech_units >= 8.0
        hard_break = speech_units >= 14.0
        if not (strong_break or soft_break or hard_break):
            continue

        cut = index + 1
        if hard_break and not (strong_break or soft_break):
            if last_space >= start and last_space >= start + (cut - start) // 2:
                cut = last_space
        segment = text[start:cut].strip()
        if segment:
            if hard_break and segment[-1] not in _LOW_LATENCY_BREAKS:
                segment += "，" if any(ord(c) > 127 for c in segment) else ","
            segments.append(segment)
        start = cut
        speech_units = 0.0
        last_space = -1

    tail = text[start:].strip()
    if tail:
        segments.append(tail)
    return segments


class KokoroRuntime:
    def __init__(self, model_dir: str, sid: int, speed: float, threads: int) -> None:
        model_dir = os.path.abspath(model_dir)
        # A warm lead engine plus continuation workers form a synthesis
        # pipeline. With the production 8-thread budget this becomes 4 + 2 + 2
        # threads: the first phrase stays fast while the next two phrases are
        # prefetched concurrently instead of starving PlayStream between them.
        thread_budget = max(2, threads)
        if thread_budget >= 8:
            lead_threads = 3
            continuation_thread_counts = [3, 2]
        elif thread_budget >= 6:
            lead_threads = 4
            continuation_thread_counts = [2]
        else:
            lead_threads = thread_budget
            continuation_thread_counts = []
        self.tts = sherpa_onnx.OfflineTts(
            self._make_config(model_dir, lead_threads)
        )
        self.continuation_tts = [
            sherpa_onnx.OfflineTts(self._make_config(model_dir, worker_threads))
            for worker_threads in continuation_thread_counts
        ]
        self.sid = sid
        self.speed = speed
        self.lock = threading.Lock()

    @staticmethod
    def _make_config(model_dir: str, threads: int):
        config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                kokoro=sherpa_onnx.OfflineTtsKokoroModelConfig(
                    model=os.path.join(model_dir, "model.int8.onnx"),
                    voices=os.path.join(model_dir, "voices.bin"),
                    tokens=os.path.join(model_dir, "tokens.txt"),
                    data_dir=os.path.join(model_dir, "espeak-ng-data"),
                    lexicon=",".join(
                        [
                            os.path.join(model_dir, "lexicon-us-en.txt"),
                            os.path.join(model_dir, "lexicon-zh.txt"),
                        ]
                    ),
                ),
                provider="cpu",
                num_threads=threads,
                debug=False,
            ),
            rule_fsts=",".join(
                [
                    os.path.join(model_dir, "phone-zh.fst"),
                    os.path.join(model_dir, "date-zh.fst"),
                    os.path.join(model_dir, "number-zh.fst"),
                ]
            ),
            max_num_sentences=1,
        )
        if not config.validate():
            raise RuntimeError("kokoro_config_invalid")
        return config

    def _to_pcm16k(self, samples: np.ndarray) -> bytes:
        samples = np.asarray(samples, dtype=np.float32)
        if samples.size == 0:
            return b""

        sample_rate = int(self.tts.sample_rate)
        if sample_rate <= 0:
            raise RuntimeError("kokoro_invalid_sample_rate")

        # Unitree AudioClient::PlayStream requires raw PCM at exactly
        # 16 kHz, mono, signed 16-bit. Resample each incremental Kokoro chunk
        # independently; the model emits contiguous speech chunks and the
        # one-sample boundary error is inaudible for this transport use.
        target_rate = 16000
        if sample_rate != target_rate and samples.size > 1:
            target_size = max(
                1, int(round(samples.size * target_rate / sample_rate))
            )
            source_positions = np.arange(samples.size, dtype=np.float64)
            target_positions = np.linspace(
                0.0, float(samples.size - 1), target_size, dtype=np.float64
            )
            samples = np.interp(
                target_positions, source_positions, samples
            ).astype(np.float32)

        pcm = np.clip(samples, -1.0, 1.0)
        pcm = (pcm * 32767.0).astype("<i2", copy=False)
        return pcm.tobytes()

    def stream_pcm16k(self, text: str, write_chunk) -> None:
        text = text.strip()
        if not text:
            raise ValueError("text_is_empty")
        if len(text.encode("utf-8")) > 4096:
            raise ValueError("text_too_long")

        def make_generation():
            generation = sherpa_onnx.GenerationConfig()
            generation.sid = self.sid
            generation.speed = self.speed
            generation.silence_scale = 0.2
            return generation

        generation = make_generation()
        emitted = 0
        keep_streaming = True

        def on_samples(samples, progress):
            nonlocal emitted, keep_streaming
            pcm = self._to_pcm16k(samples)
            if not pcm:
                return 1
            emitted += len(pcm)
            keep_streaming = write_chunk(pcm, float(progress))
            # sherpa-onnx 1.13.4 on this robot uses 1=continue, 0=stop.
            return 1 if keep_streaming else 0

        # Kokoro invokes the callback after each sentence, not every acoustic
        # frame. Keep the first phrase short enough for low startup latency, and
        # prefetch later natural phrases on warm continuation engines. Results
        # are still written strictly in text order, so prosody stays coherent
        # while synthesis overlaps the audio that is already being played.
        segments = _split_leading_text(text)
        with self.lock:
            if len(segments) == 1 or not self.continuation_tts:
                self.tts.generate(segments[0], generation, on_samples)
            else:
                lead, rest = segments[0], segments[1]
                continuation_segments = _split_continuation_text(rest)
                if not continuation_segments:
                    self.tts.generate(lead, generation, on_samples)
                else:
                    result_condition = threading.Condition()
                    results = {}
                    continuation_errors = []
                    stop_continuation = threading.Event()

                    def continuation_worker(engine, worker_index):
                        worker_count = len(self.continuation_tts)
                        for position in range(
                            worker_index, len(continuation_segments), worker_count
                        ):
                            if stop_continuation.is_set():
                                return
                            index = position + 1
                            segment = continuation_segments[position]
                            chunks = []

                            def on_continuation_samples(samples, progress):
                                if stop_continuation.is_set():
                                    return 0
                                pcm = self._to_pcm16k(samples)
                                if pcm:
                                    chunks.append(pcm)
                                return 1

                            try:
                                engine.generate(
                                    segment,
                                    make_generation(),
                                    on_continuation_samples,
                                )
                            except Exception as exc:
                                with result_condition:
                                    continuation_errors.append(exc)
                                    result_condition.notify_all()
                                stop_continuation.set()
                                return
                            with result_condition:
                                results[index] = chunks
                                result_condition.notify_all()

                    workers = [
                        threading.Thread(
                            target=continuation_worker,
                            args=(engine, index),
                            name=f"kokoro-continuation-{index}",
                            daemon=True,
                        )
                        for index, engine in enumerate(self.continuation_tts)
                    ]
                    for worker in workers:
                        worker.start()

                    self.tts.generate(lead, generation, on_samples)
                    for index in range(1, len(continuation_segments) + 1):
                        if not keep_streaming:
                            break
                        with result_condition:
                            result_condition.wait_for(
                                lambda: index in results
                                or continuation_errors
                                or not keep_streaming
                            )
                            if continuation_errors:
                                break
                            chunks = results.pop(index)
                        for chunk in chunks:
                            emitted += len(chunk)
                            keep_streaming = write_chunk(chunk, 0.0)
                            if not keep_streaming:
                                break

                    if not keep_streaming or continuation_errors:
                        stop_continuation.set()
                    for worker in workers:
                        worker.join()
                    if continuation_errors and keep_streaming:
                        raise continuation_errors[0]
        if emitted == 0:
            raise RuntimeError("kokoro_empty_audio")


class Handler(BaseHTTPRequestHandler):
    runtime: KokoroRuntime = None
    server_version = "UniRoboKokoroTTS/1.0"
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args) -> None:
        print("[kokoro] " + (fmt % args), flush=True)

    def _json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _start_pcm_stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Transfer-Encoding", "chunked")
        self.send_header("X-Sample-Rate", "16000")
        self.send_header("X-Channels", "1")
        self.send_header("X-Sample-Format", "s16le")
        self.send_header("X-Streaming", "1")
        self.end_headers()

    def _write_http_chunk(self, body: bytes) -> None:
        if not body:
            return
        self.wfile.write(f"{len(body):X}\r\n".encode("ascii"))
        self.wfile.write(body)
        self.wfile.write(b"\r\n")
        self.wfile.flush()

    def _finish_http_chunks(self) -> None:
        self.wfile.write(b"0\r\n\r\n")
        self.wfile.flush()

    def do_GET(self) -> None:
        if self.path != "/health":
            self._json(404, {"ok": False, "error": "not_found"})
            return
        self._json(
            200,
            {
                "ok": True,
                "backend": "kokoro",
                "language": "zh-en",
                "speaker_id": self.runtime.sid,
                "output": "pcm_s16le_16000_mono",
                "playback": "unitree_audioclient_playstream",
                "streaming": True,
            },
        )

    def do_POST(self) -> None:
        if self.path == "/tts":
            # The pre-streaming Web client expected this endpoint to perform
            # playback inside the Python process. Never return 200 to that
            # client with raw PCM, otherwise a rolling restart could report
            # success without producing robot-speaker audio.
            self._json(
                409,
                {
                    "ok": False,
                    "error": "legacy_tts_client_not_supported",
                    "use": "/tts/stream",
                },
            )
            return
        if self.path != "/tts/stream":
            self._json(404, {"ok": False, "error": "not_found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length <= 0 or length > 8192:
            self._json(400, {"ok": False, "error": "invalid_request_size"})
            return
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            text = payload.get("text", "") if isinstance(payload, dict) else ""
            if not isinstance(text, str):
                raise ValueError("invalid_text")
            text = text.strip()
            if not text:
                raise ValueError("text_is_empty")
            if len(text.encode("utf-8")) > 4096:
                raise ValueError("text_too_long")
        except ValueError as exc:
            self._json(400, {"ok": False, "error": str(exc)})
            return

        self._start_pcm_stream()
        connection_ok = True

        def write_chunk(pcm: bytes, progress: float) -> bool:
            nonlocal connection_ok
            if not connection_ok:
                return False
            try:
                self._write_http_chunk(pcm)
                return True
            except (BrokenPipeError, ConnectionResetError):
                connection_ok = False
                return False

        try:
            self.runtime.stream_pcm16k(text, write_chunk)
            if connection_ok:
                self._finish_http_chunks()
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as exc:
            # Headers may already be on the wire, so do not attempt to replace
            # the response with JSON. Closing the chunked response causes curl
            # to report a transport failure and the C++ side can fall back.
            print(f"[kokoro] streaming error: {type(exc).__name__}", flush=True)
            return


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--model-dir",
        default="/home/unitree/unitree_interface/tts_models/kokoro-int8-multi-lang-v1_1",
    )
    parser.add_argument("--sid", type=int, default=48)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--threads", type=int, default=4)
    args = parser.parse_args()

    # Compatibility with the pre-low-latency systemd unit already installed on
    # robots. The repository unit now passes these values explicitly, but older
    # units can be restarted without root access and still get the fast runtime.
    legacy_runtime = args.sid == 48 and args.speed == 1.0 and args.threads == 4
    previous_low_latency_runtime = (
        args.sid == 3 and args.speed == 1.2 and args.threads == 6
    )
    if legacy_runtime or previous_low_latency_runtime:
        print("[kokoro] upgrading runtime args to sid=3 speed=1.2 threads=8", flush=True)
        args.sid = 3
        args.speed = 1.2
        args.threads = 8

    runtime = KokoroRuntime(args.model_dir, args.sid, args.speed, args.threads)
    Handler.runtime = runtime
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(
        f"[kokoro] ready on http://{args.bind}:{args.port} sid={args.sid} "
        f"speed={args.speed} threads={args.threads}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
