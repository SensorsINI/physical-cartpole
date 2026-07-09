"""Live statistics for the on-chip SecLoc gate.

When firmware control is active the gate runs on the chip
(Firmware/Src/General/secloc_controller.c) and reports one telemetry flag pair
per control loop row inside the state packet: skipped_update (bit 0, the row
coasted on the held plan) and gate_skipped (bit 1, the gate was consulted and
declined). This class turns that stream into the same rolling statistics the
PC-side SeclocGate reports, so the terminal status line looks identical in
both control modes. No gate logic runs here; the chip's decisions are only
aggregated.
"""
from Control_Toolkit_ASF.Controllers.secloc_gate import SeclocStatistics


class ChipSeclocStatistics(SeclocStatistics):
    def record_telemetry(
        self, skipped_update, gate_skipped, s, target_position, time=None, target_equilibrium=1.0
    ):
        """Record one control loop row of chip telemetry.

        A row is a gate decision iff skipped_update == 0 (update) or
        gate_skipped == 1 (skip). Rows where the ref_period throttle blocked
        the gate carry neither flag and are ignored.
        """
        is_update = not skipped_update
        is_skip = bool(gate_skipped)
        if not (is_update or is_skip):
            return

        self.last_gate_evaluated = True
        self.last_gate_would_update = is_update

        ang_shift, pos_shift = self._poll_shifts(s, target_position, target_equilibrium)
        self.record_gate_poll(ang_shift, pos_shift, skipped=is_skip, time=time)
        self.record_decision(is_update)
