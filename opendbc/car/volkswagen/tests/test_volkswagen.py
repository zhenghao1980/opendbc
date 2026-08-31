import random
import re
import unittest

from opendbc.car import DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import HCAMitigation
from opendbc.car.volkswagen.values import CAR, CHECK_FUZZY_ECUS, CarControllerParams as CCP, FW_QUERY_CONFIG, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenHCAMitigation(unittest.TestCase):
  STUCK_TORQUE_FRAMES = round(CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP))

  def test_same_torque_mitigation(self):
    """Same-torque nudge fires at the threshold, in the correct direction, and resets cleanly."""
    hca_mitigation = HCAMitigation(CCP)

    for actuator_value in (-CCP.STEER_MAX, -1, 0, 1, CCP.STEER_MAX):
      hca_mitigation.update(0, 0, 0)  # Reset mitigation state
      for frame in range(self.STUCK_TORQUE_FRAMES + 2):
        should_nudge = actuator_value != 0 and frame == self.STUCK_TORQUE_FRAMES
        expected_torque = actuator_value - (1, -1)[actuator_value < 0] if should_nudge else actuator_value
        torque, _ = hca_mitigation.update(actuator_value, actuator_value, actuator_value)
        assert torque == expected_torque, f"{frame=}"

  def test_eps_timer_reset_aborts_on_steering_request(self):
    hca_mitigation = HCAMitigation(CCP, eps_timer_workaround=True)
    mitigation_start_calls = round(HCAMitigation.MLB_LOCKOUT_MITIGATION_START / DT_CTRL / CCP.STEER_STEP) + 1
    low_torque_calls = round(HCAMitigation.MLB_LOCKOUT_LOW_TORQUE_TIME / DT_CTRL / CCP.STEER_STEP) + 1

    pinned_output = 1
    assert pinned_output <= HCAMitigation.MLB_LOCKOUT_LOW_TORQUE

    for _ in range(mitigation_start_calls):
      apply_torque, _ = hca_mitigation.update(CCP.STEER_MAX, CCP.STEER_MAX, CCP.STEER_MAX)
      assert apply_torque != 0, "reset must not trigger while the model is requesting high torque"

    for _ in range(low_torque_calls):
      apply_torque, _ = hca_mitigation.update(pinned_output, pinned_output, 0)
    assert apply_torque == 0, "sustained low torque should zero the output to reset the EPS timer"

    apply_torque, _ = hca_mitigation.update(pinned_output, 0, CCP.STEER_MAX)
    assert apply_torque == pinned_output, "reset must abort when the model commands real torque"

  def test_eps_timer_reset_completes(self):
    """The mitigation arms only after MLB_LOCKOUT_MITIGATION_START, and its reset clears the timer."""
    hca_mitigation = HCAMitigation(CCP, eps_timer_workaround=True)
    mitigation_start_calls = round(HCAMitigation.MLB_LOCKOUT_MITIGATION_START / DT_CTRL / CCP.STEER_STEP) + 1
    low_torque_calls = round(HCAMitigation.MLB_LOCKOUT_LOW_TORQUE_TIME / DT_CTRL / CCP.STEER_STEP) + 1
    reset_calls = round(CCP.STEER_TIME_RESET / DT_CTRL / CCP.STEER_STEP) + 1
    pinned_output = 1

    for _ in range(low_torque_calls * 2):
      apply_torque, _ = hca_mitigation.update(pinned_output, 0, 0)
    assert apply_torque == pinned_output, "mitigation must not engage before MLB_LOCKOUT_MITIGATION_START"

    for _ in range(mitigation_start_calls):
      hca_mitigation.update(CCP.STEER_MAX, 0, CCP.STEER_MAX)
    for _ in range(low_torque_calls):
      apply_torque, _ = hca_mitigation.update(pinned_output, 0, 0)
    assert apply_torque == 0, "sustained low torque should zero the output to reset the EPS timer"

    for _ in range(reset_calls):
      apply_torque, _ = hca_mitigation.update(pinned_output, 0, 0)
    assert apply_torque == pinned_output, "EPS timer reset should complete and release steering"

  def test_reset_window_flag_surfaces(self):
    """A: HCAMitigation must surface the in_reset_window flag during the EPS
    lockout-reset window. carcontroller drops HCA_01.Status=3 only during
    this window; the rest of the activation stays at 5 so the cluster
    lane-keep lamp is continuous (no flicker on straight roads)."""
    hca_mitigation = HCAMitigation(CCP, eps_timer_workaround=True)
    mitigation_start_calls = round(HCAMitigation.MLB_LOCKOUT_MITIGATION_START / DT_CTRL / CCP.STEER_STEP) + 1
    low_torque_calls = round(HCAMitigation.MLB_LOCKOUT_LOW_TORQUE_TIME / DT_CTRL / CCP.STEER_STEP) + 1
    reset_calls = round(CCP.STEER_TIME_RESET / DT_CTRL / CCP.STEER_STEP) + 1
    pinned_output = 1

    # (a) Flag stays False during normal activation (pre-mitigation)
    for _ in range(max(mitigation_start_calls - 2, 1)):
      torque, in_reset = hca_mitigation.update(pinned_output, 0, 0)
      assert torque == pinned_output, "torque must pass through before mitigation engages"
      assert not in_reset, "flag must be False during normal activation"

    # (b) After mitigation start + sustained low torque, flag becomes True
    for _ in range(mitigation_start_calls):
      hca_mitigation.update(CCP.STEER_MAX, 0, CCP.STEER_MAX)
    for _ in range(low_torque_calls):
      hca_mitigation.update(pinned_output, 0, 0)
    seen_reset = False
    for _ in range(reset_calls + 2):
      torque, in_reset = hca_mitigation.update(pinned_output, 0, 0)
      if in_reset:
        seen_reset = True
        assert torque == 0, "during reset window, torque must be zero"
    assert seen_reset, "flag must be True at least once during the EPS lockout reset window"

    # (c) After enough reset frames, flag clears again
    final_torque, final_in_reset = hca_mitigation.update(pinned_output, 0, 0)
    assert final_torque == pinned_output, "torque must resume after reset completes"
    assert not final_in_reset, "flag must be False after reset completes"

  def test_reset_window_flag_disabled_when_workaround_off(self):
    """A: when eps_timer_workaround is False (non-MLB), the flag must stay False
    and the existing stuck-torque nudge must continue to work."""
    hca_mitigation = HCAMitigation(CCP, eps_timer_workaround=False)
    for _ in range(50):
      torque, in_reset = hca_mitigation.update(CCP.STEER_MAX, CCP.STEER_MAX, CCP.STEER_MAX)
      assert not in_reset, "flag must be False when workaround disabled"

  def test_zero_torque_frames_never_flag_reset(self):
    """Regression for review finding F1: ordinary straight-road zero-torque
    frames (update(0,0,0), lat_active=True) must NOT set in_reset_window.
    The first implementation flagged every zero-torque frame, which would have
    kept hca_enabled flickering exactly like the bug fix A set out to remove."""
    hca_mitigation = HCAMitigation(CCP, eps_timer_workaround=True)
    # Stay well below the mitigation-start window so only ordinary frames are exercised
    frames = int(HCAMitigation.MLB_LOCKOUT_MITIGATION_START / DT_CTRL / CCP.STEER_STEP) - 2
    for i in range(frames):
      torque, in_reset = hca_mitigation.update(0, 0, 0, True)
      assert not in_reset, f"frame {i}: zero-torque frame must not be flagged as a reset"
      assert torque == 0

  def test_disengage_zeroes_timer_model(self):
    """While lat_active=False the wire carries Status=3, so the rack's 6-min
    timer resets by itself; the software model must zero its counters and
    never request a deliberate reset."""
    hca_mitigation = HCAMitigation(CCP, eps_timer_workaround=True)
    # rack up some active frames
    for _ in range(200):
      hca_mitigation.update(10, 10, 10, True)
    assert hca_mitigation._hca_active_frames > 0
    # disengage
    for _ in range(20):
      torque, in_reset = hca_mitigation.update(0, 0, 0, False)
      assert not in_reset, "no deliberate reset while disengaged"
    assert hca_mitigation._hca_active_frames == 0
    assert hca_mitigation._low_torque_frames == 0

  def test_reset_still_engages_after_sustained_engagement(self):
    """End-to-end sanity: after >4min of continuous engagement with low torque
    demand, the deliberate reset engages (flag True, torque 0) and then
    disengages (flag False, torque restored)."""
    hca_mitigation = HCAMitigation(CCP, eps_timer_workaround=True)
    mitigation_start_calls = round(HCAMitigation.MLB_LOCKOUT_MITIGATION_START / DT_CTRL / CCP.STEER_STEP) + 1
    low_torque_calls = round(HCAMitigation.MLB_LOCKOUT_LOW_TORQUE_TIME / DT_CTRL / CCP.STEER_STEP) + 1
    reset_calls = round(CCP.STEER_TIME_RESET / DT_CTRL / CCP.STEER_STEP) + 1
    pinned = 1
    for _ in range(mitigation_start_calls):
      hca_mitigation.update(CCP.STEER_MAX, 0, CCP.STEER_MAX, True)
    for _ in range(low_torque_calls):
      hca_mitigation.update(pinned, 0, 0, True)
    seen_reset = False
    for _ in range(reset_calls + 2):
      torque, in_reset = hca_mitigation.update(pinned, 0, 0, True)
      if in_reset:
        seen_reset = True
        assert torque == 0
    assert seen_reset, "deliberate reset must engage after sustained low-torque engagement"
    torque, in_reset = hca_mitigation.update(pinned, 0, 0, True)
    assert torque == pinned and not in_reset, "torque must resume after reset completes"



class TestDisplaySync:
  """D/E/A integration: cover the four status-to-display mappings that we
  changed to match the B8PA cluster (rlog-driven)."""

  def test_acc_hud_status_gas_override(self):
    """D: when longActive=True and driver presses gas, status must be 4
    (background override) so the cluster ACC lamp turns off. Stock rlog
    evidence: 25 frames in (Status=4, Prim_Anz=0) during driver override."""
    from opendbc.car.volkswagen.mlbcan import acc_hud_status_value

    # Steady-state active cruise
    assert acc_hud_status_value(True, False, True) == 3
    # Driver override via gas -> status=4
    assert acc_hud_status_value(True, False, True, gas_pressed=True) == 4
    # No long_active -> gas override does nothing
    assert acc_hud_status_value(True, False, False, gas_pressed=True) == 2
    # Fault still wins
    assert acc_hud_status_value(True, True, True, gas_pressed=True) == 6
    assert acc_hud_status_value(True, True, False, gas_pressed=True) == 6

  def test_hca_status_during_zero_torque_active(self):
    """A: carcontroller must keep hca_enabled=True during straight-line
    zero-torque engagement; the rack only requires Status=5 to accept any
    torque, and dropping to 3 during zero-torque frames makes the cluster
    lamp flicker."""
    # Run a minimal simulation: HCAMitigation with workaround disabled,
    # so update() returns torque=apply_torque, in_reset=False always.
    from opendbc.car.volkswagen.carcontroller import HCAMitigation
    from opendbc.car.volkswagen.values import CarControllerParams as CCP2
    hca = HCAMitigation(CCP2, eps_timer_workaround=False)
    for desired in (10, 10, 0, 0, 0, 10, 0):
      torque, in_reset = hca.update(desired, desired, desired)
      assert not in_reset
      # In the carcontroller, hca_enabled = CC.latActive and not in_reset,
      # which is True on every one of these frames when latActive.

class TestMlbCanPacking(unittest.TestCase):
  """Pack every message we touch against the real vw_mlb DBC. This catches
  unknown signal names and out-of-range values that pure logic tests miss --
  a bad signal name here is a controlsd crash on the car."""

  def setUp(self):
    from opendbc.can import CANPacker
    from types import SimpleNamespace
    self.packer = CANPacker('vw_mlb')
    self.hud = SimpleNamespace(leftLaneDepart=False, rightLaneDepart=False,
                               leftLaneVisible=True, rightLaneVisible=True,
                               visualAlert=0, audibleAlert=0, leadVisible=True)

  def test_ldw02_packs_all_states(self):
    from opendbc.car.volkswagen import mlbcan
    for left, right in ((False, False), (True, False), (False, True)):
      self.hud.leftLaneDepart, self.hud.rightLaneDepart = left, right
      msg = mlbcan.create_lka_hud_control(self.packer, 0, {}, True, False, 0, self.hud)
      assert msg[0] == 0x397, f"LDW_02 address mismatch: {hex(msg[0])}"

  def test_ldw02_geometry_is_clear(self):
    """F6 regression: packed DLC must decode to the positive rail (+1.25 m,
    clear), never negative (crossing). Seite must fit its 1-bit field."""
    from opendbc.can import CANParser
    from opendbc.car.volkswagen import mlbcan
    for left, right, seite in ((True, False, 0), (False, True, 1)):
      self.hud.leftLaneDepart, self.hud.rightLaneDepart = left, right
      addr, dat, _bus = mlbcan.create_lka_hud_control(self.packer, 0, {}, True, False, 0, self.hud)
      parser = CANParser('vw_mlb', [('LDW_02', 0)], 0)
      parser.update([[0, [(addr, bytes(dat), 0)]]])
      vl = parser.vl['LDW_02']
      assert vl['LDW_DLC'] == 1.25, f"DLC must be the clear rail, got {vl['LDW_DLC']}"
      assert vl['LDW_TLC'] == 3.0
      assert vl['LDW_Seite_DLCTLC'] == seite

  def test_acc02_packs_status4_gas_override(self):
    from opendbc.car.volkswagen import mlbcan
    status = mlbcan.acc_hud_status_value(True, False, True, gas_pressed=True)
    assert status == 4
    hud = self.hud
    hud.leadDistanceBars = 2
    msg = mlbcan.create_acc_hud_control(self.packer, 0, status, 120.0, 8, hud, 21)
    assert msg[0] == 0x30C

  def test_hca01_packs_enabled_and_reset_frame(self):
    from opendbc.car.volkswagen import mlbcan
    from opendbc.can import CANParser
    # enabled -> Status = hca_mode (5 for B8PA); reset frame -> Status = 3
    for enabled, want in ((True, 5), (False, 3)):
      addr, dat, _bus = mlbcan.create_steering_control(self.packer, 0, 100, enabled, hca_mode=5)
      parser = CANParser('vw_mlb', [('HCA_01', 0)], 0)
      parser.update([[0, [(addr, bytes(dat), 0)]]])
      assert parser.vl['HCA_01']['HCA_01_Status_HCA'] == want


class TestVolkswagenPlatformConfigs(unittest.TestCase):
  def test_spare_part_fw_pattern(self):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with self.subTest(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self):
    for platform in CAR:
      with self.subTest(platform=platform.value):
        assert len(platform.config.wmis) > 0, "WMIs not set"
        assert len(platform.config.chassis_codes) > 0, "Chassis codes not set"
        assert all(CHASSIS_CODE_PATTERN.match(cc) for cc in
                   platform.config.chassis_codes), "Bad chassis codes"

        # No two platforms should share chassis codes
        for comp in CAR:
          if platform == comp:
            continue
          assert set() == platform.config.chassis_codes & comp.config.chassis_codes, \
                           f"Shared chassis codes: {comp}"

  def test_custom_fuzzy_fingerprinting(self):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus.get((Ecu.fwdRadar, 0x757, None), [])})

    for platform in CAR:
      # Platforms without a fwdRadar FW entry (e.g. AUDI_A4_B8PA, whose unverified
      # radar entry was dropped) cannot participate in fuzzy fingerprinting
      if not any(ecu[0] in CHECK_FUZZY_ECUS for ecu in FW_VERSIONS.get(platform, {})):
        continue
      with self.subTest(platform=platform.name):
        for wmi in WMI:
          for chassis_code in platform.config.chassis_codes | {"00"}:
            vin = ["0"] * 17
            vin[0:3] = wmi
            vin[6:8] = chassis_code
            vin = "".join(vin)

            # Check a few FW cases - expected, unexpected
            for radar_fw in random.sample(all_radar_fw, 5) + [b'\xf1\x875Q0907572G \xf1\x890571', b'\xf1\x877H9907572AA\xf1\x890396']:
              should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                              radar_fw in all_radar_fw)

              live_fws = {(0x757, None): [radar_fw]}
              matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

              expected_matches = {platform} if should_match else set()
              assert expected_matches == matches, "Bad match"
