# src/servo.py
# Front-wheel steering for WRO2025 robot (Raspberry Pi + RPi.GPIO, 50 Hz)
# Negative angle = left, Positive angle = right
# Complies with: one steering actuator (Rule 11.3), start-wait workflow (9.10–9.14), no wireless (11.6–11.12).

import time
import os
from typing import Optional

import RPi.GPIO as GPIO

try:
    import yaml  # for reading config.yaml
except ImportError:
    yaml = None


# ---------- config loader ----------
def _load_cfg():
    """
    Reads src/config.yaml if present. Falls back to sensible defaults.
    """
    defaults = {
        "servo": {
            "pin": 2,            # README default
            "freq_hz": 50,
            "center_us": 1500,
            "min_us": 600,
            "max_us": 2400,
            "deg_min": -50.0,
            "deg_max":  50.0,
            "slew_deg_per_s": 400.0,
            "failsafe_ms": 300
        },
        "turning": {
            "sharp_left":  -25.0,
            "sharp_right":  25.0,
            "max_left":    -50.0,
            "max_right":    50.0
        }
    }

    if yaml is None:
        return defaults

    # search for config.yaml (repo places it in src/)
    for cand in ("config.yaml", "./config.yaml", os.path.join(os.path.dirname(_file_), "config.yaml")):
        if os.path.exists(cand):
            with open(cand, "r") as f:
                try:
                    cfg = yaml.safe_load(f) or {}
                except Exception:
                    return defaults
            # merge shallowly
            out = defaults
            out["servo"].update((cfg.get("servo") or {}))
            out["turning"].update((cfg.get("turning") or {}))
            return out
    return defaults


# ---------- low-level servo driver ----------
class Servo:
    """
    Front-wheel steering servo driver.
    - 50 Hz PWM
    - angle interface (degrees)
    - slew limiting to avoid jerky turns
    - watchdog: stops pulses if control loop stalls (neutral/straight)
    """

    def _init_(
        self,
        pin: int,
        freq_hz: int = 50,
        center_us: int = 1500,
        min_us: int = 600,
        max_us: int = 2400,
        deg_min: float = -50.0,
        deg_max: float =  50.0,
        slew_deg_per_s: Optional[float] = 400.0,
        failsafe_ms: Optional[int] = 300,
    ):
        self.pin = pin
        self.freq = freq_hz
        self.center_us = center_us
        self.min_us = min_us
        self.max_us = max_us
        self.deg_min = deg_min
        self.deg_max = deg_max
        self.slew = slew_deg_per_s
        self.failsafe_ms = failsafe_ms

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self._pwm = GPIO.PWM(self.pin, self.freq)
        self._pwm.start(0.0)

        self._target_deg = 0.0
        self._current_deg = 0.0
        self._last_cmd_ts = time.time()
        self._prev_tick_ts = None

        # center immediately (waiting state per rules; vehicle won't move wheels unless commanded)
        self.set_deg(0.0, immediate=True)

    # ---- public API ----
    def set_deg(self, deg: float, immediate: bool = False):
        """Command an angle in degrees (− = left, + = right)."""
        self._last_cmd_ts = time.time()
        self._target_deg = self._clamp_deg(deg)
        if immediate or self.slew is None:
            self._current_deg = self._target_deg
            self._apply_deg(self._current_deg)

    def tick(self):
        """
        Call each control loop tick.
        - Applies slew limiting
        - Applies watchdog/failsafe (cuts pulses if stale)
        """
        now = time.time()

        # Watchdog: if loop stalled, stop pulses (avoids wall push / rule violations)
        if self.failsafe_ms is not None:
            if (now - self._last_cmd_ts) * 1000.0 > self.failsafe_ms:
                self._off()
                return

        if self.slew is None:
            return

        if self._prev_tick_ts is None:
            self._prev_tick_ts = now
            self._apply_deg(self._current_deg)
            return

        dt = max(0.0, now - self._prev_tick_ts)
        self._prev_tick_ts = now

        max_step = self.slew * dt
        err = self._target_deg - self._current_deg
        if abs(err) <= max_step:
            self._current_deg = self._target_deg
        else:
            self._current_deg += max_step if err > 0 else -max_step

        self._apply_deg(self._current_deg)

    def center(self):
        self.set_deg(0.0)

    def off(self):
        """Public off (freewheel)."""
        self._off()

    def close(self):
        """Release GPIO cleanly (call on shutdown/round end)."""
        try:
            self._off()
            self._pwm.stop()
        finally:
            GPIO.cleanup(self.pin)

    # ---- internals ----
    def _clamp_deg(self, d: float) -> float:
        return max(self.deg_min, min(self.deg_max, d))

    def _deg_to_us(self, d: float) -> float:
        # piecewise map so 0° -> center_us exactly
        if d >= 0:
            return self.center_us + (d / self.deg_max) * (self.max_us - self.center_us)
        else:
            return self.center_us + (d / abs(self.deg_min)) * (self.center_us - self.min_us)

    def _us_to_duty(self, pulse_us: float) -> float:
        period_us = 1_000_000.0 / self.freq
        return (pulse_us / period_us) * 100.0

    def _apply_deg(self, d: float):
        pulse = max(self.min_us, min(self.max_us, self._deg_to_us(d)))
        self._pwm.ChangeDutyCycle(self._us_to_duty(pulse))

    def _off(self):
        # 0% duty stops pulses (prevents servo heating/creep if program halts)
        self._pwm.ChangeDutyCycle(0.0)


# ---------- turning limiter (matches your spec for open + obstacle) ----------
class TurningLimiter:
    """
    Implements: during a turn, default 25°, but if |computed_angle| > 25° and < max, use it.
    Left is negative; right is positive.
    """
    def _init_(self, sharp_left=-25.0, sharp_right=25.0, max_left=-50.0, max_right=50.0):
        self.sharp_left = sharp_left
        self.sharp_right = sharp_right
        self.max_left = max_left
        self.max_right = max_right

    def apply(self, angle: float, lTurn: bool, rTurn: bool) -> float:
        if lTurn:
            # enforce left sign; pick >=25° magnitude up to max_left
            if angle >= 0:
                angle = self.sharp_left
            mag = max(abs(angle), abs(self.sharp_left))
            angle = -mag
            return max(self.max_left, angle)  # e.g., clamp to [-50, ...]
        elif rTurn:
            if angle <= 0:
                angle = self.sharp_right
            mag = max(abs(angle), abs(self.sharp_right))
            angle = +mag
            return min(self.max_right, angle)  # e.g., clamp to [..., +50]
        else:
            return angle


# ---------- convenience factory using config.yaml ----------
def make_servo_and_limiter():
    cfg = _load_cfg()
    s = cfg["servo"]; t = cfg["turning"]
    servo = Servo(
        pin=s["pin"],
        freq_hz=s["freq_hz"],
        center_us=s["center_us"],
        min_us=s["min_us"],
        max_us=s["max_us"],
        deg_min=s["deg_min"],
        deg_max=s["deg_max"],
        slew_deg_per_s=s["slew_deg_per_s"],
        failsafe_ms=s["failsafe_ms"],
    )
    limiter = TurningLimiter(
        sharp_left=t["sharp_left"],
        sharp_right=t["sharp_right"],
        max_left=t["max_left"],
        max_right=t["max_right"],
    )
    return servo, limiter


# ---------- optional: start-button helper (matches rules 9.10–9.14) ----------
def wait_for_start_button(button_pin: int, pull_up: bool = True):
    """
    Put the vehicle in waiting state until a single Start button is pressed.
    Hook this from your main program (don’t start motors/steering before this).
    """
    GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP if pull_up else GPIO.PUD_DOWN)
    # waiting state (servo centered)
    # NOTE: do NOT use any wireless or extra inputs while waiting (rules 11.6–11.12).
    while True:
        pressed = (GPIO.input(button_pin) == (GPIO.LOW if pull_up else GPIO.HIGH))
        if pressed:
            # basic debounce
            time.sleep(0.02)
            if GPIO.input(button_pin) == (GPIO.LOW if pull_up else GPIO.HIGH):
                break
        time.sleep(0.01)


# ---------- manual test ----------
if _name_ == "_main_":
    servo, limiter = make_servo_and_limiter()
    try:
        # quick sweep to verify linkage direction & limits
        for d in [-50, -25, 0, 25, 50, 0]:
            servo.set_deg(d)
            for _ in range(60):
                servo.tick()
                time.sleep(0.004)  # ~250 Hz
    except KeyboardInterrupt:
        pass
    finally:
        servo.close()
