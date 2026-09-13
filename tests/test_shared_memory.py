from __future__ import annotations

import ctypes
import unittest

from actc.shared_memory import (
    SPageFileGraphic,
    SPageFilePhysics,
    SPageFileStatic,
)


class SharedMemoryLayoutTests(unittest.TestCase):
    def test_physics_contains_core_channels(self) -> None:
        fields = {name for name, _type in SPageFilePhysics._fields_}
        self.assertIn("speedKmh", fields)
        self.assertIn("wheelAngularSpeed", fields)
        self.assertIn("localAngularVel", fields)
        self.assertIn("localVelocity", fields)
        self.assertGreater(ctypes.sizeof(SPageFilePhysics), 500)

    def test_graphics_contains_position(self) -> None:
        fields = {name for name, _type in SPageFileGraphic._fields_}
        self.assertIn("normalizedCarPosition", fields)
        self.assertIn("carCoordinates", fields)

    def test_static_contains_car_and_track(self) -> None:
        fields = {name for name, _type in SPageFileStatic._fields_}
        self.assertIn("carModel", fields)
        self.assertIn("track", fields)
        self.assertIn("tyreRadius", fields)


if __name__ == "__main__":
    unittest.main()
