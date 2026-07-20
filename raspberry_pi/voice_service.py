"""Offline voice gateway for the Raspberry Pi patrol robot.

The service consumes recognised text or acoustic events from a UART voice
module, maps them to an allow-listed vocabulary, and submits them to the local
safety arbiter.  It never writes WAVE ROVER motor JSON directly.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from typing import Iterator

import serial

from voice.intents import VoiceObservation, infer_observation
from voice.serial_module import asrpro_announcement_frames, asrpro_reply_code, parse_asrpro_byte, parse_module_line


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="NearLink robot UART voice gateway")
    parser.add_argument("--serial-port", default="disabled", help="e.g. /dev/ttyUSB1; disabled skips UART input")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument(
        "--serial-protocol",
        choices=("text", "asrpro-byte"),
        default="text",
        help="text/JSON result lines, or the robot ASRPRO one-byte event protocol",
    )
    parser.add_argument("--read-timeout", type=float, default=0.25)
    parser.add_argument(
        "--announcement-poll-s",
        type=float,
        default=0.20,
        help="Poll Pi-side proactive voice announcements at this interval in ASRPRO UART mode.",
    )
    parser.add_argument("--arbiter-url", default="http://127.0.0.1:8090")
    parser.add_argument(
        "--enable-asrpro-safety-events",
        action="store_true",
        help="Accept ASRPRO event bytes such as help. Disabled by default until the module event map is verified.",
    )
    parser.add_argument("--stdin", action="store_true", help="Read recognition text from stdin for a safe integration test")
    parser.add_argument("--text", default="", help="Submit one recognised phrase then exit")
    parser.add_argument("--microphone", action="store_true", help="Use local microphone with VAD + offline SenseVoice ASR")
    parser.add_argument("--sense-voice-model", default="")
    parser.add_argument("--tokens", default="")
    parser.add_argument("--silero-vad-model", default="")
    parser.add_argument("--microphone-device", type=int, default=None)
    parser.add_argument("--asr-threads", type=int, default=2)
    parser.add_argument("--min-confidence", type=float, default=0.72)
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


class ArbiterClient:
    def __init__(self, base_url: str, timeout: float = 2.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def submit(self, observation: VoiceObservation, source: str) -> dict[str, object]:
        payload = {
            "intent": observation.intent,
            "event": observation.event,
            "transcript": observation.transcript,
            "confidence": round(observation.confidence, 3),
            "source": source,
        }
        request = urllib.request.Request(
            f"{self.base_url}/voice/intent",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", "replace")
            try:
                result = json.loads(body)
            except json.JSONDecodeError:
                result = {"error": body or str(exc)}
            result["http_status"] = exc.code
            return result
        except urllib.error.URLError as exc:
            return {"ok": False, "error": f"arbiter unavailable: {exc.reason}"}

    def pull_announcements(self) -> list[dict[str, object]]:
        """Consume Pi-side speech prompts; this service is the only UART writer."""

        try:
            with urllib.request.urlopen(f"{self.base_url}/voice/announcements", timeout=self.timeout) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, urllib.error.HTTPError, json.JSONDecodeError):
            return []
        items = payload.get("announcements", []) if isinstance(payload, dict) else []
        return [item for item in items if isinstance(item, dict)]


def stdin_observations() -> Iterator[VoiceObservation]:
    for line in sys.stdin:
        yield infer_observation(line)


def describe_reply(reply: dict[str, object]) -> str:
    if not reply.get("ok", False):
        return str(reply.get("message") or reply.get("error") or "rejected")
    return str(reply.get("message") or reply.get("action") or "accepted")


def should_ignore_asrpro_event(observation: VoiceObservation, safety_events_enabled: bool) -> bool:
    """Quarantine unverified ASRPRO event bytes without weakening stop commands.

    The deployed ASRPRO image can emit its optional event byte repeatedly even
    when no corresponding acoustic event occurred. Its explicit patrol-stop
    command remains an intent and is therefore never filtered here.
    """

    return bool(observation.event) and not safety_events_enabled


def run_observation(
    observation: VoiceObservation,
    client: ArbiterClient,
    source: str,
    min_confidence: float,
    quiet: bool,
) -> dict[str, object] | None:
    if not observation.is_actionable:
        if not quiet and observation.transcript:
            print(f"[VOICE] ignored transcript={observation.transcript!r}", flush=True)
        return None
    if observation.confidence < min_confidence:
        if not quiet:
            print(f"[VOICE] ignored low confidence={observation.confidence:.2f}", flush=True)
        return None
    reply = client.submit(observation, source)
    if not quiet:
        print(
            "[VOICE] "
            f"intent={observation.intent or '-'} event={observation.event or '-'} "
            f"text={observation.transcript!r} -> {describe_reply(reply)}",
            flush=True,
        )
    return reply


def run_serial_mode(args: argparse.Namespace, client: ArbiterClient) -> None:
    source = f"uart:{args.serial_port}:{args.serial_protocol}"
    print(f"[VOICE] listening {source} @ {args.baudrate}; arbiter={args.arbiter_url}", flush=True)
    while True:
        try:
            with serial.Serial(
                args.serial_port,
                args.baudrate,
                timeout=args.read_timeout,
                write_timeout=args.read_timeout,
                dsrdtr=False,
                rtscts=False,
            ) as device:
                device.dtr = False
                device.rts = False
                next_announcement_poll = 0.0
                while True:
                    raw = device.read(1) if args.serial_protocol == "asrpro-byte" else device.readline()
                    if not raw:
                        now = time.monotonic()
                        if args.serial_protocol == "asrpro-byte" and now >= next_announcement_poll:
                            next_announcement_poll = now + max(0.05, args.announcement_poll_s)
                            for announcement in client.pull_announcements():
                                frames = asrpro_announcement_frames(announcement)
                                if not frames:
                                    continue
                                for frame in frames:
                                    device.write(frame)
                                device.flush()
                                if not args.quiet:
                                    print(
                                        f"[VOICE] announced kind={announcement.get('kind', '')} "
                                        f"id={announcement.get('id', '')} bytes={frames!r}",
                                        flush=True,
                                    )
                        continue
                    observation = (
                        parse_asrpro_byte(raw)
                        if args.serial_protocol == "asrpro-byte"
                        else parse_module_line(raw)
                    )
                    if args.serial_protocol == "asrpro-byte" and should_ignore_asrpro_event(
                        observation, args.enable_asrpro_safety_events
                    ):
                        if not args.quiet:
                            print(
                                f"[VOICE] ignored unverified ASRPRO event={observation.event} "
                                f"text={observation.transcript!r}",
                                flush=True,
                            )
                        continue
                    reply = run_observation(observation, client, source, args.min_confidence, args.quiet)
                    if args.serial_protocol == "asrpro-byte" and reply is not None:
                        response_code = asrpro_reply_code(observation, reply)
                        if response_code is not None:
                            device.write(bytes((response_code,)))
                            device.flush()
        except serial.SerialException as exc:
            print(f"[VOICE] UART unavailable: {exc}; retrying in 2s", file=sys.stderr, flush=True)
            time.sleep(2.0)


def main() -> int:
    args = parse_args()
    client = ArbiterClient(args.arbiter_url)
    if args.text:
        run_observation(infer_observation(args.text), client, "cli", args.min_confidence, args.quiet)
        return 0

    if args.serial_port != "disabled":
        run_serial_mode(args, client)

    if args.stdin:
        for observation in stdin_observations():
            run_observation(observation, client, "stdin", args.min_confidence, args.quiet)
        return 0

    if args.microphone:
        if not (args.sense_voice_model and args.tokens and args.silero_vad_model):
            print("microphone mode needs --sense-voice-model, --tokens, and --silero-vad-model", file=sys.stderr)
            return 2
        from voice.sherpa_asr import microphone_observations

        try:
            for observation in microphone_observations(
                args.sense_voice_model,
                args.tokens,
                args.silero_vad_model,
                num_threads=args.asr_threads,
                device=args.microphone_device,
            ):
                run_observation(observation, client, "microphone:sensevoice", args.min_confidence, args.quiet)
        except RuntimeError as exc:
            print(f"[VOICE] {exc}", file=sys.stderr)
            return 2
        return 0

    print("Set --serial-port, --microphone, or use --stdin/--text.", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
