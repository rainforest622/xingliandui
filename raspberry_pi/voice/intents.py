"""Small, deterministic intent layer between speech recognition and the robot."""

from __future__ import annotations

from dataclasses import dataclass
import re


INTENT_PATROL_START = "patrol_start"
INTENT_PATROL_PAUSE = "patrol_pause"
INTENT_PATROL_RESUME = "patrol_resume"
INTENT_PATROL_STOP = "patrol_stop"
INTENT_STATUS_REPORT = "status_report"
INTENT_ENVIRONMENT_REPORT = "environment_report"
INTENT_DISTANCE_REPORT = "distance_report"
INTENT_PATROL_REPORT = "patrol_report"
INTENT_BATTERY_REPORT = "battery_report"
INTENT_ALARM_CLEAR_REQUEST = "alarm_clear_request"

EVENT_HELP = "help"
EVENT_ALARM_SOUND = "alarm_sound"
EVENT_IMPACT = "impact"

SUPPORTED_INTENTS = {
    INTENT_PATROL_START,
    INTENT_PATROL_PAUSE,
    INTENT_PATROL_RESUME,
    INTENT_PATROL_STOP,
    INTENT_STATUS_REPORT,
    INTENT_ENVIRONMENT_REPORT,
    INTENT_DISTANCE_REPORT,
    INTENT_PATROL_REPORT,
    INTENT_BATTERY_REPORT,
    INTENT_ALARM_CLEAR_REQUEST,
}
SUPPORTED_EVENTS = {EVENT_HELP, EVENT_ALARM_SOUND, EVENT_IMPACT}


@dataclass(frozen=True)
class VoiceObservation:
    """A normalised local recognition result, ready for safety arbitration."""

    transcript: str
    confidence: float = 1.0
    intent: str = ""
    event: str = ""

    @property
    def is_actionable(self) -> bool:
        return bool(self.intent or self.event)


def normalize_phrase(value: str) -> str:
    """Remove punctuation and spacing without changing the spoken content."""

    return re.sub(r"[\s,，。！？!?、:：;；\-—_]+", "", value.strip().lower())


def _contains_any(text: str, phrases: tuple[str, ...]) -> bool:
    return any(phrase in text for phrase in phrases)


def infer_observation(text: str, confidence: float = 1.0) -> VoiceObservation:
    """Map a Chinese transcript to the deliberately small command vocabulary.

    Speech recognition can vary slightly in wording.  The matching stays local,
    explainable, and intentionally cannot emit raw motor commands.
    """

    raw = text.strip()
    normal = normalize_phrase(raw)
    bounded_confidence = max(0.0, min(1.0, float(confidence)))

    if _contains_any(normal, ("救命", "帮帮我", "帮忙", "有人吗", "着火")):
        return VoiceObservation(raw, bounded_confidence, event=EVENT_HELP)
    if _contains_any(normal, ("撞击", "碰撞", "爆裂", "砸落")):
        return VoiceObservation(raw, bounded_confidence, event=EVENT_IMPACT)
    if _contains_any(normal, ("异常报警声", "报警声", "设备报警", "报警")):
        return VoiceObservation(raw, bounded_confidence, event=EVENT_ALARM_SOUND)

    if _contains_any(normal, ("暂停巡检", "暂停任务", "暂停")):
        return VoiceObservation(raw, bounded_confidence, intent=INTENT_PATROL_PAUSE)
    if _contains_any(normal, ("继续巡检", "恢复巡检", "继续任务", "继续")):
        return VoiceObservation(raw, bounded_confidence, intent=INTENT_PATROL_RESUME)
    if _contains_any(normal, ("停止巡检", "停止任务", "停车", "停下", "停止")):
        return VoiceObservation(raw, bounded_confidence, intent=INTENT_PATROL_STOP)
    if _contains_any(normal, ("开始巡检", "开始自动巡检", "执行巡检", "启动巡检")):
        return VoiceObservation(raw, bounded_confidence, intent=INTENT_PATROL_START)
    if _contains_any(normal, ("报告温湿度", "温湿度", "环境数据", "环境状态")):
        return VoiceObservation(raw, bounded_confidence, intent=INTENT_ENVIRONMENT_REPORT)
    if _contains_any(normal, ("前方距离", "报告距离", "障碍距离", "距离状态")):
        return VoiceObservation(raw, bounded_confidence, intent=INTENT_DISTANCE_REPORT)
    if _contains_any(normal, ("巡检进度", "巡检报告", "任务进度", "路线进度")):
        return VoiceObservation(raw, bounded_confidence, intent=INTENT_PATROL_REPORT)
    if _contains_any(normal, ("报告电量", "当前电量", "电池电量", "电池状态")):
        return VoiceObservation(raw, bounded_confidence, intent=INTENT_BATTERY_REPORT)
    if _contains_any(normal, ("报告环境", "报告状态", "系统状态", "当前状态")):
        return VoiceObservation(raw, bounded_confidence, intent=INTENT_STATUS_REPORT)
    if _contains_any(normal, ("解除报警", "取消报警")):
        return VoiceObservation(raw, bounded_confidence, intent=INTENT_ALARM_CLEAR_REQUEST)
    return VoiceObservation(raw, bounded_confidence)


def observation_for_event(event: str, transcript: str = "", confidence: float = 1.0) -> VoiceObservation:
    """Normalise a hardware event code without allowing unrecognised events."""

    normal = normalize_phrase(event).replace("_", "")
    aliases = {
        "help": EVENT_HELP,
        "helpevent": EVENT_HELP,
        "求助": EVENT_HELP,
        "alarm": EVENT_ALARM_SOUND,
        "alarmsound": EVENT_ALARM_SOUND,
        "报警": EVENT_ALARM_SOUND,
        "impact": EVENT_IMPACT,
        "撞击": EVENT_IMPACT,
    }
    return VoiceObservation(
        transcript.strip(),
        max(0.0, min(1.0, float(confidence))),
        event=aliases.get(normal, ""),
    )
