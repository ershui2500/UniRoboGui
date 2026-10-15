#!/usr/bin/env python3
"""Local TTS content preservation and interrupted HTTP stream regression tests."""
import http.client
import importlib.util
import json
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import Mock, patch
from http.server import ThreadingHTTPServer

# Protocol tests need no installed model or native inference dependencies.
numpy_stub = types.ModuleType('numpy')
numpy_stub.ndarray = object
sys.modules['numpy'] = numpy_stub
sys.modules['sherpa_onnx'] = types.SimpleNamespace(GenerationConfig=Mock)
spec = importlib.util.spec_from_file_location(
    'local_tts', Path(__file__).resolve().parents[1] / 'scripts/kokoro_tts_server.py')
tts = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tts)


class LocalTtsTests(unittest.TestCase):
    def test_model_file_selection_preserves_int8_and_supports_fp32(self):
        for quantized in [True, False]:
            kokoro_config = Mock()
            with patch.object(tts.os.path, 'isfile', return_value=quantized), patch.multiple(
                tts.sherpa_onnx, create=True,
                OfflineTtsKokoroModelConfig=kokoro_config,
                OfflineTtsModelConfig=Mock(), OfflineTtsConfig=Mock(),
            ):
                tts.KokoroRuntime._make_config('/model', 4)
            self.assertEqual(kokoro_config.call_args.kwargs['model'],
                             '/model/model.int8.onnx' if quantized else '/model/model.onnx')

    def test_non_finite_audio_is_rejected_before_pcm_conversion(self):
        runtime = tts.KokoroRuntime.__new__(tts.KokoroRuntime)
        with patch.multiple(
            tts.np, create=True, float32=object,
            asarray=Mock(return_value=types.SimpleNamespace(size=3)),
            isfinite=Mock(return_value=types.SimpleNamespace(all=lambda: False)),
        ):
            with self.assertRaisesRegex(RuntimeError, 'kokoro_non_finite_audio'):
                runtime._to_pcm16k([float('nan'), float('inf'), 0.0])

    def test_single_engine_preserves_entire_utterance(self):
        for text in ['你好', '你好，机器人。请完整播放最后一句。',
                     '没有标点的长句也必须完整保留到最后一个字',
                     'Hello robot. 今天是2026年10月15日。']:
            runtime = tts.KokoroRuntime.__new__(tts.KokoroRuntime)
            runtime.tts = Mock()
            runtime.continuation_tts = []
            runtime.sid, runtime.speed = 3, 1.2
            runtime.lock = threading.Lock()
            runtime._to_pcm16k = lambda samples: b'\x01\x00'
            def generate(value, config, callback):
                self.assertEqual(value, text)
                callback(None, 1.0)
            runtime.tts.generate.side_effect = generate
            output = []
            runtime.stream_pcm16k(text, lambda pcm, progress: output.append(pcm) or True)
            self.assertEqual(output, [b'\x01\x00'])

    def test_http_stream_success_and_failure(self):
        class Runtime:
            sid = 3
            def stream_pcm16k(self, text, emit):
                if text != 'fail_before':
                    emit(b'\x01\x00', 0.5)
                if text.startswith('fail_'):
                    raise RuntimeError('test_synthesis_failure')
                emit(b'\x02\x00', 1.0)
        class Handler(tts.Handler):
            runtime = Runtime()
            def log_message(self, *args):
                pass
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            for text in ['ok', 'fail_before', 'fail_after']:
                connection = http.client.HTTPConnection(*server.server_address, timeout=2)
                try:
                    connection.request('POST', '/tts/stream', json.dumps({'text': text}),
                                       {'Content-Type': 'application/json'})
                    response = connection.getresponse()
                    self.assertEqual(response.status, 200)
                    if text == 'ok':
                        self.assertEqual(response.read(), b'\x01\x00\x02\x00')
                    else:
                        # A truncated stream must fail immediately, never look
                        # like a successful partial utterance or wait for timeout.
                        with self.assertRaises(http.client.IncompleteRead):
                            response.read()
                finally:
                    connection.close()
        finally:
            server.shutdown()
            server.server_close()
            worker.join()


if __name__ == '__main__':
    unittest.main()
