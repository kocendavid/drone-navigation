"""Map decoding and distance transforms execute in a separate process."""

import numpy as np
from .mapping import RayMap
from .centering import center_geometry


class MapGeometry:
    def __init__(self, settings):
        self.settings = settings
        self.cached = None
        self.geometry_key = None

    def __call__(self, item):
        width, height = item["width"], item["height"]
        chars = item["grid"]
        old = self.cached
        confirmed = item.get("confirmed_walls", "0" * (width * height))
        geometry_key = (chars, confirmed)
        if old is not None and geometry_key == getattr(self, "geometry_key", None):
            model, clearance, body_free, safe = (
                old[0],
                old[1],
                old[2],
                old[3],
            )
            center_clearance, centerline = old[4], old[5]
        else:
            model = RayMap(**self.settings.map_options())
            model.grid = np.array(
                [{"?": 0, ".": 1, "#": 2, "b": 3}[c] for c in chars], dtype=np.uint8
            ).reshape(height, width)
            if len(confirmed) != width * height or set(confirmed) - set("01"):
                raise ValueError("Invalid wall confirmation mask")
            model.confirmed_walls = np.array(list(confirmed)).reshape(height, width) == "1"
            if np.any(model.confirmed_walls & (model.grid != 2)):
                raise ValueError("Confirmed walls must remain occupied")
            clearance, body_free, safe = model.geometry()
            center_clearance, centerline = center_geometry(model, clearance, safe)
        self.geometry_key = geometry_key
        self.cached = model, clearance, body_free, safe, center_clearance, centerline
        return self.cached
