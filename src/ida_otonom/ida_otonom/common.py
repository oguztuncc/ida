import json
import math
import os
import time
from pathlib import Path

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - only used before ROS env is sourced
    get_package_share_directory = None


PACKAGE_NAME = "ida_otonom"
DEFAULT_MISSION_NAME = "mission.json"


def now_ts() -> float:
    return time.time()


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def normalize_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = (
        math.sin(dp / 2.0) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    )
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)

    y = math.sin(dl) * math.cos(p2)
    x = (
        math.cos(p1) * math.sin(p2)
        - math.sin(p1) * math.cos(p2) * math.cos(dl)
    )

    brg = math.degrees(math.atan2(y, x))
    return (brg + 360.0) % 360.0


def to_json(data: dict) -> str:
    return json.dumps(data, ensure_ascii=False)


def from_json(text: str) -> dict:
    return json.loads(text)


def package_share_path() -> Path:
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory(PACKAGE_NAME))
        except Exception:
            pass

    return Path(__file__).resolve().parent


def default_mission_file() -> str:
    return str(package_share_path() / "missions" / DEFAULT_MISSION_NAME)


def resolve_mission_file(configured_path: str) -> Path:
    if configured_path:
        return Path(configured_path).expanduser()

    return Path(default_mission_file())


def default_record_dir() -> str:
    configured = os.environ.get("IDA_RECORD_DIR")
    if configured:
        return str(Path(configured).expanduser())

    return str(Path.home() / ".ros" / PACKAGE_NAME / "records")
