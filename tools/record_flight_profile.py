"""Replay/follow the passive timing log; stream captured samples and export graphs."""

import argparse
import csv
import json
import mmap
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.flight_profile import FlightProfile, number
from navigation.flight_profile_ui import ProfileServer

PROJECT = Path(__file__).resolve().parents[1]
KINDS = (b'"isaac_truth"', b'"px4_position"', b'"px4_setpoint"', b'"reference"')


def recording_clock_offset(recording):
    """Reuse a saved clock pair, or calibrate a recorder verified to be live on this boot."""
    anchor_path = recording.parent / "wall_clock.json"
    try:
        anchor = json.loads(anchor_path.read_text(encoding="utf-8"))
        unix, monotonic = anchor["unix_s"], anchor["monotonic_s"]
        if number(unix) and number(monotonic):
            return unix - monotonic
    except (OSError, ValueError, KeyError, TypeError):
        pass
    try:
        status_path = PROJECT / "logs" / "timing_diagnostics.json"
        status = json.loads(status_path.read_text(encoding="utf-8"))
        unix, monotonic = time.time(), time.perf_counter()
        wall_age = unix - status_path.stat().st_mtime
        clock_age = monotonic - status["updated_wall_s"]
        if (
            status.get("recording")
            and Path(status["log_file"]).resolve() == recording.resolve()
            and 0 <= wall_age <= 20
            and 0 <= clock_age <= 20
            and abs(wall_age - clock_age) < 1
        ):
            anchor = dict(unix_s=unix, monotonic_s=monotonic)
            try:
                anchor_path.write_text(json.dumps(anchor) + "\n", encoding="utf-8")
            except OSError:
                pass
            return unix - monotonic
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def latest_run_offset(path):
    """Locate the last bridge session without JSON-decoding earlier multi-GB runs."""
    if not path.stat().st_size:
        return 0
    with path.open("rb") as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
        index = data.rfind(b'"isaac_truth"')
        if index < 0:
            return 0
        start = data.rfind(b"\n", 0, index) + 1
        end = data.find(b"\n", index)
        if end < 0:
            return 0
        record = json.loads(data[start:end])
        first = data.find(record["session"].encode("ascii"))
        return data.rfind(b"\n", 0, first) + 1


def plot_profile(path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return []
    x = [float(r["simulation_s"]) for r in rows]
    moving = [
        float(r["simulation_s"])
        for r in rows
        if r["phase"] in ("NAVIGATING", "PLANNING", "SETTLING")
    ]
    armed = [float(r["simulation_s"]) for r in rows if r["armed"] == "True"]
    metadata = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
    outputs = []
    windows = [("", max(x[0], min(armed) - 3) if armed else x[0], x[-1])]
    if moving:
        windows.append(("_navigation", max(x[0], min(moving) - 3), min(x[-1], max(moving) + 12)))
    for suffix, start, end in windows:
        fig, axes = plt.subplots(2, 1, figsize=(13, 7.5), sharex=True, constrained_layout=True)
        for ax, series in zip(
            axes,
            (
                (
                    ("simulation_height_m", "Simulator truth", "#147d64", "-"),
                    ("estimated_height_m", "PX4 estimate · common floor", "#d47b13", "-"),
                    ("commanded_height_m", "Commanded height", "#5666c2", "--"),
                ),
                (
                    ("actual_speed_m_s", "Simulator actual speed", "#147d64", "-"),
                    ("estimated_speed_m_s", "PX4 estimated speed", "#d47b13", "-"),
                    ("expected_speed_m_s", "Expected speed · reference", "#5666c2", "--"),
                    ("selected_speed_m_s", "Selected speed ceiling", "#a34e94", ":"),
                ),
            ),
        ):
            for key, label, color, style in series:
                y = [float(r[key]) if r[key] else float("nan") for r in rows]
                ax.plot(x, y, style, color=color, label=label, linewidth=1.5)
            ax.grid(alpha=0.22)
            ax.legend(loc="upper right", fontsize=8)
            ax.set_ylim(bottom=0)
        axes[0].axhline(0.8, color="#b34b4b", linewidth=1, alpha=0.7)
        axes[0].axhline(1.2, color="#b34b4b", linewidth=1, alpha=0.7)
        axes[0].set_ylim(0, max(1.3, axes[0].get_ylim()[1]))
        axes[0].legend(loc="lower left", fontsize=8)
        axes[0].set_ylabel("Height above mapping floor (m)")
        axes[1].set_ylabel("Horizontal speed (m/s)")
        axes[1].set_xlabel("Isaac simulation time (s) · one instantaneous sample per second")
        axes[1].set_xlim(start, max(end, start + 1))
        previous = None
        for r in rows:
            phase = r["phase"]
            if phase == "PAUSED" and previous and previous != "PAUSED":
                stamp = float(r["simulation_s"])
                for ax in axes:
                    ax.axvline(stamp, color="#b34b4b", alpha=0.55, linewidth=1)
                if start <= stamp <= end:
                    axes[1].annotate(
                        "PAUSED",
                        (stamp, 0.03),
                        xycoords=("data", "axes fraction"),
                        rotation=90,
                        color="#b34b4b",
                        fontsize=8,
                    )
            if phase:
                previous = phase
        fig.suptitle(
            "Height and speed through navigation" if suffix else "Flight height and speed",
            fontsize=15,
        )
        fig.text(
            0.01,
            -0.025,
            f"Source-time alignment · floor Z = {metadata['floor_world_z_m']:.4f} m · gaps stay missing · red horizontal lines: mapper height limits",
            fontsize=8,
        )
        output = path.with_name(path.stem + suffix + ".png")
        temporary = output.with_suffix(".tmp.png")
        fig.savefig(temporary, dpi=145, bbox_inches="tight")
        plt.close(fig)
        temporary.replace(output)
        outputs.append(output)
    return outputs


def publish_status(profile, recording, position, *, live=True, graphs=()):
    """Publish flushed CSV samples independently of optional PNG rendering."""
    if not profile.files:
        return False
    status = dict(
        recording=live,
        updated_unix_s=time.time(),
        source=str(recording),
        source_offset_bytes=position,
        samples=profile.samples,
        latest=profile.latest,
        csv=str(profile.files[-1]),
        graphs=list(map(str, graphs)),
        floor_world_z_m=profile.floor_z,
    )
    temporary = profile.folder / "status.tmp"
    temporary.write_text(json.dumps(status, indent=2) + "\n", encoding="utf-8")
    # Windows can briefly deny replacement while a viewer or scanner reads the file.
    for attempt in range(4):
        try:
            temporary.replace(profile.folder / "status.json")
            return True
        except PermissionError:
            if attempt < 3:
                time.sleep(0.01 * (attempt + 1))
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "recording", type=Path, nargs="?", help="Defaults to active timing recorder"
    )
    parser.add_argument("--output", type=Path, default=PROJECT / "logs" / "flight_profiles")
    parser.add_argument("--floor-z", type=float, help="Mapping floor used by this recording")
    parser.add_argument(
        "--follow",
        action="store_true",
        help="Continue recording and stream chart updates as samples are captured",
    )
    parser.add_argument(
        "--latest-run", action="store_true", help="Replay only the most recent bridge session"
    )
    parser.add_argument("--plot-csv", type=Path, help="Only redraw an existing profile CSV")
    parser.add_argument(
        "--ui-port", type=int, default=8788, help="Local read-only graph viewer port when following"
    )
    args = parser.parse_args()
    if args.plot_csv:
        for path in plot_profile(args.plot_csv):
            print(path, flush=True)
        return
    if args.follow and args.recording is None:
        # The timing recorder already produces aligned CSV samples. The live
        # viewer follows those files directly, including recorder/run changes.
        viewer = ProfileServer(
            PROJECT / "logs",
            args.ui_port,
            status_path=PROJECT / "logs" / "timing_diagnostics.json",
        )
        try:
            while True:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            viewer.close()
        return
    floor = args.floor_z
    if floor is None:
        floor = json.loads((PROJECT / "config" / "navigation_settings.json").read_text())[
            "floor_z_m"
        ]
    recording = args.recording
    if recording is None:
        recording = Path(
            json.loads((PROJECT / "logs" / "timing_diagnostics.json").read_text())["log_file"]
        )
    profile = FlightProfile(args.output, floor, recording_clock_offset(recording))
    viewer = ProfileServer(args.output, args.ui_port) if args.follow else None
    last_publish = last_source_check = 0
    published_samples = -1
    position = latest_run_offset(recording) if args.latest_run else 0
    began = time.monotonic()
    try:
        with recording.open("rb") as stream:
            stream.seek(position)
            while True:
                line = stream.readline()
                if line and line.endswith(b"\n"):
                    position = stream.tell()
                    if any(kind in line for kind in KINDS):
                        try:
                            profile.observe(json.loads(line))
                        except (ValueError, KeyError, TypeError, IndexError) as error:
                            print(f"Unavailable input: {error}", file=sys.stderr, flush=True)
                    # Also publish during catch-up or a continuously busy source log.
                    now = time.monotonic()
                    if (
                        args.follow
                        and profile.samples != published_samples
                        and now - last_publish >= 0.2
                    ):
                        if publish_status(profile, recording, position):
                            published_samples = profile.samples
                        last_publish = now
                    continue
                # A partial last line remains unread until the writer finishes it.
                stream.seek(position)
                if not args.follow:
                    break
                now = time.monotonic()
                if args.recording is None and now - last_source_check >= 5:
                    last_source_check = now
                    # The independent timing service may itself be restarted.
                    try:
                        active = Path(
                            json.loads((PROJECT / "logs" / "timing_diagnostics.json").read_text())[
                                "log_file"
                            ]
                        )
                    except (OSError, ValueError, KeyError):
                        active = recording
                    if active != recording and active.is_file():
                        stream.close()
                        stream = active.open("rb")
                        recording = active
                        profile.wall_clock_offset_s = recording_clock_offset(recording)
                        position = latest_run_offset(active) if args.latest_run else 0
                        stream.seek(position)
                        continue
                if profile.files and (
                    profile.samples != published_samples or now - last_publish >= 5
                ):
                    if publish_status(profile, recording, position):
                        published_samples = profile.samples
                    last_publish = now
                if began is not None and profile.files:
                    print(
                        f"Following {recording}; replay took {now - began:.1f}s; CSV {profile.files[-1]}",
                        flush=True,
                    )
                    began = None
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if "stream" in locals():
            stream.close()
        profile.finish()
        if profile.files:
            paths = plot_profile(profile.files[-1])
            publish_status(profile, recording, position, live=False, graphs=paths)
            for path in paths:
                print(path, flush=True)
        if viewer:
            viewer.close()


if __name__ == "__main__":
    main()
