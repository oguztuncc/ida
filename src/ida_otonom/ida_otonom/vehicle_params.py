"""
Arac profili ve guvenlik zarfi yonetimi.

Tum node'lar arac boyutlarini ve guvenlik mesafelerini bu modulden
alinmasi hedeflenir. Boylece simulasyon ve gercek arac arasindaki
tutarsizlik tek yerden cozulur.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from .common import get_package_share_directory

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class VehicleProfile:
    """SDF ve gercek donanim ile uyumlu arac fiziksel profili."""

    name: str = "ida_katamaran"
    length_m: float = 1.11
    beam_m: float = 0.76
    thruster_track_m: float = 0.60
    lidar_x_m: float = 0.0
    lidar_z_m: float = 0.45
    mass_kg: float = 32.6
    max_linear_speed_mps: float = 0.35
    max_angular_speed_radps: float = 0.6


@dataclass(frozen=True)
class SafetyEnvelope:
    """Guvenlik mesafeleri; planner ve sensor node'lar tarafindan kullanilir."""

    side_clearance_m: float = 0.35
    front_clearance_m: float = 0.85
    emergency_stop_m: float = 0.45
    danger_m: float = 0.60
    avoid_start_m: float = 5.0
    min_pass_gap_m: float = 1.50
    collision_distance_m: float = 1.20

    # Turetilmis degerler (validate sonrasi hesaplanir)
    front_path_half_width_m: float = field(default=0.0, repr=False)


def _resolve_profile_path(path_or_name: str) -> Path:
    """Profil ismini veya yolunu cozumle."""
    path = Path(path_or_name).expanduser()
    if path.is_absolute() or path.exists():
        return path

    # Paket paylasim dizininde ara
    share_dir: Optional[Path] = None
    try:
        share_dir = Path(get_package_share_directory("ida_otonom"))
    except Exception:
        pass

    if share_dir is not None:
        candidate = share_dir / "config" / "vehicle_profiles" / f"{path_or_name}.yaml"
        if candidate.exists():
            return candidate
        candidate = share_dir / "config" / "vehicle_profiles" / path_or_name
        if candidate.exists():
            return candidate

    # Calisma dizininden ara (gelistirme ortami)
    ws_candidate = (
        Path(__file__).resolve().parent.parent.parent
        / "config"
        / "vehicle_profiles"
        / f"{path_or_name}.yaml"
    )
    if ws_candidate.exists():
        return ws_candidate

    raise FileNotFoundError(
        f"Arac profili bulunamadi: {path_or_name}"
    )


def load_vehicle_profile(
    path_or_name: str = "ida_katamaran",
) -> Tuple[VehicleProfile, SafetyEnvelope]:
    """YAML dosyasindan VehicleProfile ve SafetyEnvelope yukle."""
    path = _resolve_profile_path(path_or_name)
    with open(path, "r", encoding="utf-8") as fh:
        data: Dict[str, Any] = yaml.safe_load(fh) or {}

    vehicle_data = data.get("vehicle", {})
    safety_data = data.get("safety", {})

    profile = VehicleProfile(
        name=str(vehicle_data.get("name", "ida_katamaran")),
        length_m=float(vehicle_data.get("length_m", 1.11)),
        beam_m=float(vehicle_data.get("beam_m", 0.76)),
        thruster_track_m=float(vehicle_data.get("thruster_track_m", 0.60)),
        lidar_x_m=float(vehicle_data.get("lidar_x_m", 0.0)),
        lidar_z_m=float(vehicle_data.get("lidar_z_m", 0.45)),
        mass_kg=float(vehicle_data.get("mass_kg", 32.6)),
        max_linear_speed_mps=float(
            vehicle_data.get("max_linear_speed_mps", 0.35)
        ),
        max_angular_speed_radps=float(
            vehicle_data.get("max_angular_speed_radps", 0.6)
        ),
    )

    envelope = SafetyEnvelope(
        side_clearance_m=float(safety_data.get("side_clearance_m", 0.35)),
        front_clearance_m=float(safety_data.get("front_clearance_m", 0.85)),
        emergency_stop_m=float(safety_data.get("emergency_stop_m", 0.45)),
        danger_m=float(safety_data.get("danger_m", 0.60)),
        avoid_start_m=float(safety_data.get("avoid_start_m", 5.0)),
        min_pass_gap_m=float(safety_data.get("min_pass_gap_m", 1.50)),
        collision_distance_m=float(
            safety_data.get("collision_distance_m", 1.20)
        ),
    )

    return profile, envelope


def derive_safety_params(
    profile: VehicleProfile, envelope: SafetyEnvelope
) -> Dict[str, float]:
    """Profil ve zarftan node'lara iletilecek turetilmis parametreleri uret."""
    front_path_half_width_m = max(
        profile.beam_m / 2.0 + envelope.side_clearance_m,
        envelope.front_clearance_m,
    )

    safe_clearance_m = max(
        envelope.side_clearance_m,
        profile.beam_m / 2.0 + 0.1,
    )

    return {
        "vehicle_width_m": profile.beam_m,
        "vehicle_length_m": profile.length_m,
        "vehicle_half_width_m": profile.beam_m / 2.0,
        "vehicle_half_length_m": profile.length_m / 2.0,
        "front_path_half_width_m": front_path_half_width_m,
        "safe_clearance_m": safe_clearance_m,
        "collision_distance_m": envelope.collision_distance_m,
        "danger_distance_m": envelope.danger_m,
        "emergency_stop_distance_m": envelope.emergency_stop_m,
        "avoid_start_distance_m": envelope.avoid_start_m,
        "min_pass_gap_width_m": envelope.min_pass_gap_m,
        "side_safety_margin_m": envelope.side_clearance_m,
    }


def validate_envelope(
    profile: VehicleProfile, envelope: SafetyEnvelope
) -> List[str]:
    """Guvenlik parametrelerinin tutarliligini kontrol et."""
    errors: List[str] = []

    min_gap_required = profile.beam_m + 2.0 * envelope.side_clearance_m
    if envelope.min_pass_gap_m < min_gap_required:
        errors.append(
            f"min_pass_gap_m ({envelope.min_pass_gap_m:.2f}) >= "
            f"beam_m + 2*side_clearance_m ({min_gap_required:.2f}) "
            f"saglanmiyor!"
        )

    if envelope.danger_m < envelope.emergency_stop_m:
        errors.append(
            f"danger_m ({envelope.danger_m:.2f}) >= "
            f"emergency_stop_m ({envelope.emergency_stop_m:.2f}) "
            f"saglanmiyor!"
        )

    if envelope.avoid_start_m < envelope.danger_m:
        errors.append(
            f"avoid_start_m ({envelope.avoid_start_m:.2f}) >= "
            f"danger_m ({envelope.danger_m:.2f}) saglanmiyor!"
        )

    if envelope.front_clearance_m < profile.beam_m / 2.0:
        errors.append(
            f"front_clearance_m ({envelope.front_clearance_m:.2f}) >= "
            f"beam_m/2 ({profile.beam_m / 2.0:.2f}) saglanmiyor!"
        )

    if envelope.side_clearance_m < 0.05:
        errors.append(
            f"side_clearance_m ({envelope.side_clearance_m:.2f}) "
            f"cok kucuk (minimum 0.05)."
        )

    return errors


def log_profile_info(
    logger_obj: Any,
    profile: VehicleProfile,
    envelope: SafetyEnvelope,
) -> None:
    """Yuklenen profili INFO seviyesinde logla."""
    logger_obj.info(
        f"Arac profili yuklendi: {profile.name} "
        f"(L={profile.length_m:.2f}m, B={profile.beam_m:.2f}m, "
        f"T={profile.thruster_track_m:.2f}m)"
    )
    logger_obj.info(
        f"Guvenlik zarfi: danger={envelope.danger_m:.2f}m, "
        f"emergency_stop={envelope.emergency_stop_m:.2f}m, "
        f"avoid_start={envelope.avoid_start_m:.2f}m, "
        f"collision={envelope.collision_distance_m:.2f}m, "
        f"min_pass_gap={envelope.min_pass_gap_m:.2f}m"
    )
