"""The graph viewer serves only recorded assets and exposes no flight actions."""

import json
from pathlib import Path
import tempfile
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from navigation.flight_profile_ui import ProfileServer


class FlightProfileUITests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.folder = Path(temporary.name)
        self.server = ProfileServer(self.folder, port=0)
        self.addCleanup(self.server.close)

    def status(self, **values):
        (self.folder / "status.json").write_text(json.dumps(values), encoding="utf-8")

    def read(self, route):
        return urlopen(self.server.url + route, timeout=3)

    def test_waiting_state_and_live_assets_with_csv_download(self):
        with self.read("/api/profile") as response:
            self.assertFalse(json.load(response)["available"])
        csv = self.folder / "flight.csv"
        csv.write_text("simulation_s,actual_speed_m_s,phase\n1,0.7,NAVIGATING\n")
        full = self.folder / "flight.png"
        navigation = self.folder / "flight_navigation.png"
        full.write_bytes(b"full image")
        navigation.write_bytes(b"navigation image")
        self.status(
            recording=True,
            updated_unix_s=time.time(),
            csv=str(csv),
            graphs=[str(full), str(navigation)],
            latest=dict(simulation_s=12),
        )
        with self.read("/api/profile") as response:
            status = json.load(response)
            self.assertTrue(status["live"])
            self.assertTrue(status["navigation_available"])
            self.assertEqual(status["simulation_s"], 1)
        for route, content in (
            ("/plot?view=navigation&v=1", b"navigation image"),
            ("/plot?view=full", b"full image"),
            ("/samples.csv", csv.read_bytes()),
        ):
            with self.read(route) as response:
                self.assertEqual(response.read(), content)
                self.assertEqual(response.headers["Cache-Control"], "no-store")
                if route == "/samples.csv":
                    self.assertIn("attachment", response.headers["Content-Disposition"])

    def test_chart_samples_available_before_any_png_with_gaps_and_nonfinite_values(self):
        csv = self.folder / "live.csv"
        csv.write_text(
            "simulation_s,simulation_height_m,actual_speed_m_s,phase,armed\n"
            "1,0.9,0.7,NAVIGATING,True\n2,,nan,PAUSED,False\n"
            "3,1.0,",  # An unfinished write must not invent a data point.
            encoding="utf-8",
        )
        self.status(csv=str(csv), recording=True, updated_unix_s=time.time())
        with self.read("/api/samples") as response:
            data = json.load(response)
        self.assertTrue(data["available"])
        self.assertTrue(data["navigation_available"])
        self.assertTrue(data["live"])
        self.assertEqual(len(data["samples"]), 2)
        self.assertEqual(data["samples"][0]["simulation_height_m"], 0.9)
        self.assertIs(data["samples"][0]["armed"], True)
        self.assertIsNone(data["samples"][1]["simulation_height_m"])
        self.assertIsNone(data["samples"][1]["actual_speed_m_s"])
        self.assertEqual(data["samples"][1]["phase"], "PAUSED")

    def test_capture_events_update_on_csv_append_and_reset_without_png_or_timer(self):
        def event(stream):
            self.assertEqual(stream.readline(), b"event: profile\n")
            data = json.loads(stream.readline().removeprefix(b"data: "))
            self.assertEqual(stream.readline(), b"\n")
            return data

        with self.read("/api/events") as stream:
            self.assertEqual(stream.headers["Content-Type"], "text/event-stream; charset=utf-8")
            self.assertFalse(event(stream)["available"])
            csv = self.folder / "live.csv"
            csv.write_text("simulation_s,actual_speed_m_s\n1,0.7\n", encoding="utf-8")
            self.status(csv=str(csv), recording=True, updated_unix_s=time.time())
            first = event(stream)
            self.assertEqual(first["samples"][0]["actual_speed_m_s"], 0.7)
            began = time.monotonic()
            with csv.open("a", encoding="utf-8") as output:
                output.write("2,0.8\n")
            second = event(stream)
            self.assertEqual(second["samples"][-1]["simulation_s"], 2)
            self.assertNotEqual(first["version"], second["version"])
            self.assertLess(time.monotonic() - began, 2)
            reset = self.folder / "reset.csv"
            reset.write_text("simulation_s,actual_speed_m_s\n0,0.0\n", encoding="utf-8")
            self.status(csv=str(reset), recording=False)
            third = event(stream)
            self.assertNotEqual(second["run"], third["run"])
            self.assertEqual(len(third["samples"]), 1)
            self.assertFalse(third["live"])

    def test_chart_engine_is_local_and_viewer_uses_capture_events(self):
        with self.read("/") as response:
            html = response.read().decode()
        self.assertIn('new EventSource("/api/events")', html)
        self.assertIn("/vendor/uPlot.iife.min.js", html)
        for route, mime in (
            ("/vendor/uPlot.iife.min.js", "text/javascript"),
            ("/vendor/uPlot.min.css", "text/css"),
        ):
            with self.read(route) as response:
                self.assertTrue(response.headers["Content-Type"].startswith(mime))
                self.assertGreater(len(response.read()), 100)

    def test_stale_or_missing_recorder_keeps_saved_graph_but_cannot_appear_live(self):
        self.status(recording=True, updated_unix_s=time.time() - 60)
        with self.read("/api/profile") as response:
            self.assertFalse(json.load(response)["live"])
        for route in ("/plot?view=navigation", "/samples.csv", "/plot?view=../../README.md"):
            with self.assertRaises(HTTPError) as error:
                self.read(route)
            self.assertEqual(error.exception.code, 404)

    def test_assets_outside_profile_folder_and_flight_posts_are_refused(self):
        with tempfile.TemporaryDirectory() as outside:
            secret = Path(outside) / "private.csv"
            secret.write_text("private")
            self.status(csv=str(secret))
            with self.assertRaises(HTTPError) as error:
                self.read("/samples.csv")
            self.assertEqual(error.exception.code, 404)
        with self.assertRaises(HTTPError) as error:
            urlopen(
                Request(self.server.url + "/api/command", data=b'{"action":"takeoff"}'), timeout=3
            )
        self.assertEqual(error.exception.code, 501)


if __name__ == "__main__":
    unittest.main()
