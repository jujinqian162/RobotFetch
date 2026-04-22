from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float
    deadband: float
    derivative_alpha: float


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class PIDController:
    def __init__(self, cfg: PIDConfig) -> None:
        self._cfg = cfg
        self._integral = 0.0
        self._last_error: float | None = None
        self._last_derivative = 0.0
        self._last_time_s: float | None = None

    def set_config(self, cfg: PIDConfig) -> None:
        self._cfg = cfg

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = None
        self._last_derivative = 0.0
        self._last_time_s = None

    def update(self, error: float, now_s: float) -> float:
        cfg = self._cfg
        if abs(error) <= cfg.deadband:
            error = 0.0

        dt = 0.0
        if self._last_time_s is not None:
            dt = max(1e-6, now_s - self._last_time_s)

        derivative = 0.0
        if self._last_error is not None and dt > 0.0:
            raw_derivative = (error - self._last_error) / dt
            alpha = _clamp(cfg.derivative_alpha, 0.0, 1.0)
            derivative = alpha * raw_derivative + (1.0 - alpha) * self._last_derivative

        p_term = cfg.kp * error
        d_term = cfg.kd * derivative

        integral_candidate = self._integral
        if dt > 0.0 and cfg.ki != 0.0:
            integral_candidate = _clamp(
                self._integral + error * dt,
                -cfg.integral_limit,
                cfg.integral_limit,
            )

        i_term = cfg.ki * integral_candidate
        output_unclamped = p_term + i_term + d_term
        output = _clamp(output_unclamped, -cfg.output_limit, cfg.output_limit)

        saturated_high = output >= cfg.output_limit and error > 0.0
        saturated_low = output <= -cfg.output_limit and error < 0.0
        if not (saturated_high or saturated_low):
            self._integral = integral_candidate

        self._last_error = error
        self._last_derivative = derivative
        self._last_time_s = now_s
        return output
