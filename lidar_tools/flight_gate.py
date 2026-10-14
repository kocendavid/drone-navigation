"""Fresh observed-surface gate for the bounded flat-room flight experiment."""

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class GateDecision:
    allowed: bool
    state: str
    reason: str
    distance_m: float | None = None

    def report(self):
        return asdict(self)


class ClearanceGate:
    """Validate live worker records; repeated reports cannot refresh scan age."""

    stop_m = 2.2  # 2 m observed-surface threshold plus a 0.2 m test reserve.

    def __init__(self, session):
        self.session = session
        self.stamp = self.scan_progress_wall = self.generated_wall = None
        self.cloud_wall = self.clock_wall = None
        self.clear_scans = 0
        self.latest = GateDecision(False, "UNKNOWN", "Waiting for live LiDAR worker")

    def invalidate(self, reason):
        self.clear_scans = 0
        self.latest = GateDecision(False, "UNKNOWN", reason)

    def observe(self, envelope, now):
        try:
            if envelope["session"] != self.session:
                raise ValueError("Different LiDAR worker session")
            generated = envelope["generated_monotonic_s"]
            if (
                not isinstance(generated, (float, int))
                or not math.isfinite(generated)
                or not 0 <= now - generated <= 0.6
            ):
                raise ValueError("LiDAR pipe record is buffered, stale or from a future clock")
            self.generated_wall = generated
            report = envelope["report"]
            if report["body_frame"] != "chassis" or report["world_frame"] != "world":
                raise ValueError("Unexpected LiDAR coordinate frame")
            state = report["state"]
            if state == "UNKNOWN":
                self.invalidate("LiDAR UNKNOWN: " + str(report.get("reason", "No usable scan")))
                return
            if state not in ("CLEAR", "HOLD") or report["floor_ready"] is not True:
                raise ValueError("Invalid clearance state or uncalibrated floor")
            for field in ("cloud_age_wall_s", "clock_age_wall_s"):
                age = report[field]
                if (
                    not isinstance(age, (float, int))
                    or not math.isfinite(age)
                    or not 0 <= age <= 1.0
                ):
                    raise ValueError("LiDAR cloud or simulation clock is stale")
            self.cloud_wall = generated - report["cloud_age_wall_s"]
            self.clock_wall = generated - report["clock_age_wall_s"]
            expected = dict(
                hold_m=2.0,
                min_range_m=1.0,
                max_range_m=30.0,
                forward_min_m=0.2,
                forward_max_m=15.0,
                half_width_m=0.5,
                body_z_min_m=-0.3,
                body_z_max_m=0.5,
                floor_margin_m=0.12,
            )
            limits = report["limits"]
            if any(
                not math.isclose(float(limits[key]), value, rel_tol=0.0, abs_tol=1e-8)
                for key, value in expected.items()
            ) or not math.isfinite(float(limits["floor_z_world_m"])):
                raise ValueError("LiDAR corridor or filter settings changed")
            stamp, distance = report["stamp_ns"], report["forward_distance_m"]
            if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
                raise ValueError("Missing positive cloud acquisition timestamp")
            if (
                isinstance(distance, bool)
                or not isinstance(distance, (float, int))
                or not math.isfinite(distance)
                or not 0.2 <= distance <= 15.0
            ):
                raise ValueError("Invalid observed forward distance")
            if (state == "HOLD") != (distance <= 2.0) or report["corridor_points"] <= 0:
                raise ValueError("Clearance state disagrees with observed surface")
            nearest = report["nearest_body_xyz_m"]
            if (
                len(nearest) != 3
                or not all(math.isfinite(v) for v in nearest)
                or not math.isclose(nearest[0], distance, abs_tol=1e-6)
            ):
                raise ValueError("Nearest point disagrees with distance")
            if abs(nearest[1]) > 0.500001 or not -0.300001 <= nearest[2] <= 0.500001:
                raise ValueError("Nearest point is outside the expected body corridor")
            if self.stamp is not None and stamp < self.stamp:
                self.stamp = stamp
                self.scan_progress_wall = None
                self.invalidate("Cloud clock moved backward; reconfirm fresh scans")
                return
            advancing = self.stamp is None or stamp > self.stamp
            self.stamp = stamp
            if advancing:
                if (
                    self.scan_progress_wall is not None
                    and generated - self.scan_progress_wall > 1.0
                ):
                    self.clear_scans = 0
                self.scan_progress_wall = generated
            if distance <= self.stop_m:
                self.clear_scans = 0
                self.latest = GateDecision(
                    False,
                    "HOLD",
                    f"Observed surface {distance:.2f} m ahead; stop trigger {self.stop_m:.2f} m",
                    distance,
                )
            else:
                if advancing:
                    self.clear_scans += 1
                confirmed = self.clear_scans >= 3
                self.latest = GateDecision(
                    confirmed,
                    "CLEAR" if confirmed else "UNKNOWN",
                    f"Observed surface {distance:.2f} m ahead; "
                    + (
                        "three fresh scans confirmed"
                        if confirmed
                        else f"confirming scans {self.clear_scans}/3"
                    ),
                    distance,
                )
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.invalidate("Invalid LiDAR record: " + str(exc))

    def decision(self, now):
        if self.generated_wall is None or not 0 <= now - self.generated_wall <= 0.8:
            return GateDecision(False, "UNKNOWN", "LiDAR worker record missing or stale")
        if self.latest.state == "UNKNOWN":
            return self.latest
        if any(
            wall is None or not 0 <= now - wall <= 1.0
            for wall in (self.cloud_wall, self.clock_wall)
        ):
            return GateDecision(False, "UNKNOWN", "LiDAR cloud or simulation clock is stale")
        if self.scan_progress_wall is None or not 0 <= now - self.scan_progress_wall <= 1.0:
            return GateDecision(False, "UNKNOWN", "Cloud timestamp is not progressing")
        return self.latest
