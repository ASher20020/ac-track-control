from __future__ import annotations

import ctypes
import os
import subprocess
import time
from abc import ABC, abstractmethod
from ctypes import wintypes
from pathlib import Path


XUSB_GAMEPAD_A = 0x1000
XUSB_GAMEPAD_B = 0x2000
XUSB_GAMEPAD_X = 0x4000
XUSB_GAMEPAD_Y = 0x8000


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class InputBackend(ABC):
    name = "base"

    @abstractmethod
    def apply(self, steer: float, throttle: float, brake: float) -> None:
        raise NotImplementedError

    def close(self) -> None:
        self.apply(0.0, 0.0, 0.0)

    def tap_button(self, button_mask: int, duration_s: float = 0.12) -> None:
        pass


class DryRunInput(InputBackend):
    name = "dry-run"

    def __init__(self) -> None:
        self.steer = 0.0
        self.throttle = 0.0
        self.brake = 0.0

    def apply(self, steer: float, throttle: float, brake: float) -> None:
        self.steer = clamp(steer, -1.0, 1.0)
        self.throttle = clamp(throttle, 0.0, 1.0)
        self.brake = clamp(brake, 0.0, 1.0)


if os.name == "nt":
    ULONG_PTR = wintypes.WPARAM

    class KEYBDINPUT(ctypes.Structure):
        _fields_ = [
            ("wVk", wintypes.WORD),
            ("wScan", wintypes.WORD),
            ("dwFlags", wintypes.DWORD),
            ("time", wintypes.DWORD),
            ("dwExtraInfo", ULONG_PTR),
        ]

    class MOUSEINPUT(ctypes.Structure):
        _fields_ = [
            ("dx", wintypes.LONG),
            ("dy", wintypes.LONG),
            ("mouseData", wintypes.DWORD),
            ("dwFlags", wintypes.DWORD),
            ("time", wintypes.DWORD),
            ("dwExtraInfo", ULONG_PTR),
        ]

    class HARDWAREINPUT(ctypes.Structure):
        _fields_ = [
            ("uMsg", wintypes.DWORD),
            ("wParamL", wintypes.WORD),
            ("wParamH", wintypes.WORD),
        ]

    class _INPUTUNION(ctypes.Union):
        _fields_ = [
            ("ki", KEYBDINPUT),
            ("mi", MOUSEINPUT),
            ("hi", HARDWAREINPUT),
        ]

    class INPUT(ctypes.Structure):
        _anonymous_ = ("u",)
        _fields_ = [
            ("type", wintypes.DWORD),
            ("u", _INPUTUNION),
        ]


class KeyboardInput(InputBackend):
    """Keyboard output using time-based PWM to approximate analog commands."""

    name = "keyboard"
    INPUT_KEYBOARD = 1
    KEYEVENTF_EXTENDEDKEY = 0x0001
    KEYEVENTF_KEYUP = 0x0002
    VK_LEFT = 0x25
    VK_UP = 0x26
    VK_RIGHT = 0x27
    VK_DOWN = 0x28

    def __init__(
        self,
        pedal_period_s: float = 0.10,
        steering_period_s: float = 0.20,
        deadband: float = 0.03,
    ) -> None:
        if os.name != "nt":
            raise RuntimeError("Keyboard input is Windows-only")
        self.pedal_period_s = pedal_period_s
        self.steering_period_s = steering_period_s
        self.deadband = deadband
        self._pressed: dict[int, bool] = {}
        self._started_at = time.perf_counter()
        self._send_input = ctypes.windll.user32.SendInput
        self._send_input.argtypes = (
            wintypes.UINT,
            ctypes.POINTER(INPUT),
            ctypes.c_int,
        )
        self._send_input.restype = wintypes.UINT

    def _send_key(self, virtual_key: int, is_down: bool) -> None:
        if self._pressed.get(virtual_key, False) == is_down:
            return
        flags = 0
        if virtual_key in (self.VK_LEFT, self.VK_RIGHT, self.VK_UP, self.VK_DOWN):
            flags |= self.KEYEVENTF_EXTENDEDKEY
        if not is_down:
            flags |= self.KEYEVENTF_KEYUP
        event = INPUT(
            type=self.INPUT_KEYBOARD,
            ki=KEYBDINPUT(
                wVk=virtual_key,
                wScan=0,
                dwFlags=flags,
                time=0,
                dwExtraInfo=0,
            ),
        )
        if self._send_input(1, ctypes.byref(event), ctypes.sizeof(INPUT)) != 1:
            raise ctypes.WinError()
        self._pressed[virtual_key] = is_down

    @staticmethod
    def _pwm_is_on(now: float, magnitude: float, period_s: float) -> bool:
        phase = (now % period_s) / period_s
        return phase < clamp(magnitude, 0.0, 1.0)

    def apply(self, steer: float, throttle: float, brake: float) -> None:
        now = time.perf_counter()
        steer = clamp(steer, -1.0, 1.0)
        throttle = clamp(throttle, 0.0, 1.0)
        brake = clamp(brake, 0.0, 1.0)

        if abs(steer) < self.deadband:
            self._send_key(self.VK_LEFT, False)
            self._send_key(self.VK_RIGHT, False)
        else:
            steer_on = self._pwm_is_on(now, abs(steer), self.steering_period_s)
            self._send_key(self.VK_LEFT, steer_on and steer < 0.0)
            self._send_key(self.VK_RIGHT, steer_on and steer > 0.0)

        if throttle >= brake:
            brake = 0.0
        else:
            throttle = 0.0

        self._send_key(
            self.VK_UP,
            throttle >= self.deadband
            and self._pwm_is_on(now, throttle, self.pedal_period_s),
        )
        self._send_key(
            self.VK_DOWN,
            brake >= self.deadband
            and self._pwm_is_on(now, brake, self.pedal_period_s),
        )

    def close(self) -> None:
        for key in (self.VK_LEFT, self.VK_RIGHT, self.VK_UP, self.VK_DOWN):
            self._send_key(key, False)


class VJoyInput(InputBackend):
    """Direct vJoy output. The device must expose axes X, Y and Z."""

    name = "vjoy"
    SCALE = 32768
    CENTER = 16384

    class JOYSTICK_POSITION(ctypes.Structure):
        _pack_ = 1
        _fields_ = [
            ("bDevice", ctypes.c_ubyte),
            ("wThrottle", ctypes.c_int32),
            ("wRudder", ctypes.c_int32),
            ("wAileron", ctypes.c_int32),
            ("wAxisX", ctypes.c_int32),
            ("wAxisY", ctypes.c_int32),
            ("wAxisZ", ctypes.c_int32),
            ("wAxisXRot", ctypes.c_int32),
            ("wAxisYRot", ctypes.c_int32),
            ("wAxisZRot", ctypes.c_int32),
            ("wSlider", ctypes.c_int32),
            ("wDial", ctypes.c_int32),
            ("wWheel", ctypes.c_int32),
            ("wAxisVX", ctypes.c_int32),
            ("wAxisVY", ctypes.c_int32),
            ("wAxisVZ", ctypes.c_int32),
            ("wAxisVBRX", ctypes.c_int32),
            ("wAxisVBRY", ctypes.c_int32),
            ("wAxisVBRZ", ctypes.c_int32),
            ("lButtons", ctypes.c_uint32),
            ("bHats", ctypes.c_uint32),
            ("bHatsEx1", ctypes.c_uint32),
            ("bHatsEx2", ctypes.c_uint32),
            ("bHatsEx3", ctypes.c_uint32),
        ]

    def __init__(self, device_id: int = 1, dll_path: str | None = None) -> None:
        if os.name != "nt":
            raise RuntimeError("vJoy input is Windows-only")
        path = Path(
            dll_path
            or r"C:\Program Files\vJoy\x64\vJoyInterface.dll"
        )
        if not path.exists():
            raise RuntimeError(
                f"vJoyInterface.dll not found at {path}. Install and configure vJoy first."
            )
        self.device_id = device_id
        self.dll = ctypes.WinDLL(str(path))
        self.dll.AcquireVJD.argtypes = (ctypes.c_uint,)
        self.dll.AcquireVJD.restype = wintypes.BOOL
        self.dll.RelinquishVJD.argtypes = (ctypes.c_uint,)
        self.dll.RelinquishVJD.restype = wintypes.BOOL
        self.dll.UpdateVJD.argtypes = (
            ctypes.c_uint,
            ctypes.POINTER(self.JOYSTICK_POSITION),
        )
        self.dll.UpdateVJD.restype = wintypes.BOOL
        if not self.dll.AcquireVJD(self.device_id):
            raise RuntimeError(f"Cannot acquire vJoy device {self.device_id}")
        self._acquired = True
        self._position = self.JOYSTICK_POSITION()
        self._position.bDevice = device_id

    def apply(self, steer: float, throttle: float, brake: float) -> None:
        steer = clamp(steer, -1.0, 1.0)
        throttle = clamp(throttle, 0.0, 1.0)
        brake = clamp(brake, 0.0, 1.0)
        self._position.wAxisX = int((steer + 1.0) * self.CENTER)
        self._position.wAxisY = int(throttle * self.SCALE)
        self._position.wAxisZ = int(brake * self.SCALE)
        if not self.dll.UpdateVJD(self.device_id, ctypes.byref(self._position)):
            raise RuntimeError("UpdateVJD failed")

    def close(self) -> None:
        if getattr(self, "_acquired", False):
            self.apply(0.0, 0.0, 0.0)
            self.dll.RelinquishVJD(self.device_id)
            self._acquired = False


class VGamepadInput(InputBackend):
    """Virtual Xbox 360 output through vgamepad or the bundled ViGEm bridge."""

    name = "vgamepad"

    def __init__(self) -> None:
        if os.name != "nt":
            raise RuntimeError("vgamepad input is Windows-only")
        self.process = None
        self.gamepad = None
        self._button_mask = 0
        self._active_button_mask = 0
        self._button_release_at = 0.0
        try:
            import vgamepad as vg
        except ImportError:
            vg = None
        if vg is not None:
            self.vg = vg
            self.gamepad = vg.VX360Gamepad()
            return

        project_root = Path(__file__).resolve().parents[1]
        script = project_root / "tools" / "vigem_bridge.ps1"
        dll = project_root / "tools" / "Nefarius.ViGEm.Client.dll"
        if not script.exists() or not dll.exists():
            raise RuntimeError("The ViGEm bridge files are missing from tools/")
        powershell = (
            Path(os.environ.get("SystemRoot", r"C:\Windows"))
            / "System32"
            / "WindowsPowerShell"
            / "v1.0"
            / "powershell.exe"
        )
        if not powershell.exists():
            raise RuntimeError("Windows PowerShell was not found")
        self.process = subprocess.Popen(
            [
                str(powershell),
                "-NoLogo",
                "-NoProfile",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(script),
                "-DllPath",
                str(dll),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        ready = self.process.stdout.readline().strip() if self.process.stdout else ""
        if ready != "READY":
            error = self.process.stderr.read().strip() if self.process.stderr else ""
            self.process.terminate()
            raise RuntimeError(f"ViGEm bridge failed to start: {ready} {error}".strip())

    def apply(self, steer: float, throttle: float, brake: float) -> None:
        steer = clamp(steer, -1.0, 1.0)
        throttle = clamp(throttle, 0.0, 1.0)
        brake = clamp(brake, 0.0, 1.0)
        now = time.perf_counter()
        if now >= self._button_release_at:
            self._button_mask = 0

        if self.gamepad is not None:
            self.gamepad.left_joystick_float(
                x_value_float=steer,
                y_value_float=0.0,
            )
            self.gamepad.right_trigger_float(value_float=throttle)
            self.gamepad.left_trigger_float(value_float=brake)
            if self._button_mask != self._active_button_mask:
                button = self.vg.XUSB_BUTTON.XUSB_GAMEPAD_A
                if self._button_mask & XUSB_GAMEPAD_A:
                    self.gamepad.press_button(button=button)
                elif self._active_button_mask & XUSB_GAMEPAD_A:
                    self.gamepad.release_button(button=button)
                self._active_button_mask = self._button_mask
            self.gamepad.update()
            return

        if self.process is None or self.process.poll() is not None:
            raise RuntimeError("ViGEm bridge process exited")
        steer_value = int(steer * 32767)
        throttle_value = int(throttle * 255)
        brake_value = int(brake * 255)
        assert self.process.stdin is not None
        self.process.stdin.write(
            f"{steer_value},{throttle_value},{brake_value},{self._button_mask}\n"
        )
        self.process.stdin.flush()

    def tap_button(self, button_mask: int, duration_s: float = 0.12) -> None:
        self._button_mask = int(button_mask)
        self._button_release_at = time.perf_counter() + max(0.05, duration_s)

    def close(self) -> None:
        if self.gamepad is not None:
            try:
                self.gamepad.reset()
                self.gamepad.update()
            finally:
                self.gamepad = None
            return
        if getattr(self, "process", None) is None:
            return
        if self.process.poll() is None:
            try:
                self.apply(0.0, 0.0, 0.0)
                time.sleep(0.10)
                if self.process.stdin is not None:
                    self.process.stdin.write("QUIT\n")
                    self.process.stdin.flush()
                self.process.wait(timeout=2.0)
            except Exception:
                self.process.terminate()
        if self.process.stdout is not None:
            self.process.stdout.close()
        if self.process.stderr is not None:
            self.process.stderr.close()
        if self.process.stdin is not None:
            self.process.stdin.close()


def create_input_backend(name: str) -> InputBackend:
    normalized = name.lower()
    if normalized == "dry-run":
        return DryRunInput()
    if normalized == "keyboard":
        return KeyboardInput()
    if normalized == "vjoy":
        return VJoyInput()
    if normalized == "vgamepad":
        return VGamepadInput()
    raise ValueError(f"Unknown input backend: {name}")


def focus_ac_window() -> bool:
    """Best-effort focus of the Assetto Corsa window."""
    if os.name != "nt":
        return False
    user32 = ctypes.windll.user32
    exact_matches: list[int] = []
    partial_matches: list[int] = []
    enum_proc_type = ctypes.WINFUNCTYPE(
        wintypes.BOOL,
        wintypes.HWND,
        wintypes.LPARAM,
    )

    def callback(hwnd: int, _lparam: int) -> bool:
        if not user32.IsWindowVisible(hwnd):
            return True
        length = user32.GetWindowTextLengthW(hwnd)
        if length <= 0:
            return True
        buffer = ctypes.create_unicode_buffer(length + 1)
        user32.GetWindowTextW(hwnd, buffer, length + 1)
        title = buffer.value.casefold()
        if title == "assetto corsa":
            exact_matches.append(hwnd)
            return False
        if "assetto corsa" in title or "assettocorsa" in title:
            partial_matches.append(hwnd)
        return True

    user32.EnumWindows(enum_proc_type(callback), 0)
    matches = exact_matches or partial_matches
    if not matches:
        return False
    hwnd = matches[0]
    foreground = user32.GetForegroundWindow()
    target_thread = user32.GetWindowThreadProcessId(hwnd, None)
    foreground_thread = user32.GetWindowThreadProcessId(foreground, None)
    current_thread = ctypes.windll.kernel32.GetCurrentThreadId()
    attached_target = False
    attached_foreground = False
    try:
        if target_thread and target_thread != current_thread:
            attached_target = bool(
                user32.AttachThreadInput(current_thread, target_thread, True)
            )
        if foreground_thread and foreground_thread != current_thread:
            attached_foreground = bool(
                user32.AttachThreadInput(
                    current_thread,
                    foreground_thread,
                    True,
                )
            )
        user32.ShowWindow(hwnd, 9)
        user32.BringWindowToTop(hwnd)
        user32.SetForegroundWindow(hwnd)
        user32.SetFocus(hwnd)
    finally:
        if attached_foreground:
            user32.AttachThreadInput(
                current_thread,
                foreground_thread,
                False,
            )
        if attached_target:
            user32.AttachThreadInput(current_thread, target_thread, False)
    return user32.GetForegroundWindow() == hwnd
