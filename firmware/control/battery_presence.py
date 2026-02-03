# control/battery_presence.py
#
# Battery attach/remove detector with debounce.
# Works with mocks now; refine thresholds once hardware arrives.

import time

class BatteryPresence:
    def __init__(self, *, vbat_present_v=1.5, vbat_removed_v=0.8, debounce_ms=800):
        self.vbat_present_v = float(vbat_present_v)
        self.vbat_removed_v = float(vbat_removed_v)
        self.debounce_ms = int(debounce_ms)

        self.present = False
        self._candidate = None
        self._since_ms = time.ticks_ms()

    def update(self, vbat_v):
        """
        Returns (changed, present_now)
        """
        v = float(vbat_v)
        now = time.ticks_ms()

        # raw decision
        raw_present = v >= self.vbat_present_v if not self.present else v >= self.vbat_removed_v

        if raw_present == self.present:
            self._candidate = None
            return (False, self.present)

        # candidate change
        if self._candidate is None:
            self._candidate = raw_present
            self._since_ms = now
            return (False, self.present)

        if self._candidate != raw_present:
            # bounced
            self._candidate = raw_present
            self._since_ms = now
            return (False, self.present)

        # stable long enough?
        if time.ticks_diff(now, self._since_ms) >= self.debounce_ms:
            self.present = raw_present
            self._candidate = None
            return (True, self.present)

        return (False, self.present)
