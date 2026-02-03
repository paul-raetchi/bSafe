# control/charge_control.py
#
# Deterministic CC/CV decision logic for the charger state machine.

import time


class ChargeControl:
    def __init__(self):
        self._phase_start_ms = time.ticks_ms()

    def reset_phase_timers(self):
        self._phase_start_ms = time.ticks_ms()

    def _seconds_in_phase(self):
        return time.ticks_diff(time.ticks_ms(), self._phase_start_ms) // 1000

    def decide(self, *,
               state: str,
               vbat_v: float,
               ibat_a: float,
               target_vbat_v: float,
               cap_ah: float,
               charge_a: float,
               settings: dict,
               config_module):
        """
        Returns decision dict with keys:
          - next_state: "PRECONDITION" | "CC" | "CV" | None
          - set_current_a: float or None
          - done: bool
          - stop_charging: bool
        """
        res = {
            "next_state": None,
            "set_current_a": None,
            "done": False,
            "stop_charging": False,
        }

        if target_vbat_v is None or charge_a is None:
            return res

        vbat = float(vbat_v) if vbat_v is not None else 0.0
        ibat = float(ibat_a) if ibat_a is not None else 0.0

        precond_exit_ratio = float(getattr(config_module, "PRECOND_EXIT_RATIO", 0.90))
        precond_c_rate = float(getattr(config_module, "PRECOND_C_RATE", 0.10))
        cv_hyst_v = float(getattr(config_module, "CV_HYST_V", 0.02))
        term_c_rate = float(getattr(config_module, "TERM_C_RATE", 0.05))
        cv_min_time_s = int(getattr(config_module, "CV_MIN_TIME_S", 60))

        precond_exit_v = target_vbat_v * precond_exit_ratio
        term_current_a = abs(charge_a) * term_c_rate

        if state == "PRECONDITION":
            res["set_current_a"] = abs(charge_a) * precond_c_rate
            if vbat >= precond_exit_v:
                res["next_state"] = "CC"

        elif state == "CC":
            res["set_current_a"] = abs(charge_a)
            if vbat >= (target_vbat_v - cv_hyst_v):
                res["next_state"] = "CV"

        elif state == "CV":
            res["set_current_a"] = abs(charge_a)
            if self._seconds_in_phase() >= cv_min_time_s and abs(ibat) <= term_current_a:
                res["done"] = True
                res["stop_charging"] = True

        return res
