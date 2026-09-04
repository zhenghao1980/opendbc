import random
import re
import unittest

from opendbc.car import DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from opendbc.car.volkswagen.carcontroller import HCAMitigation
from opendbc.car.volkswagen.mlbcan import create_lka_hud_control as mlb_create_lka_hud_control
from opendbc.car.volkswagen.mqbcan import LANE_KEEP_STANDSTILL_M_S, create_lka_hud_control as mqb_create_lka_hud_control
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
        assert hca_mitigation.update(actuator_value, actuator_value, actuator_value) == expected_torque, f"{frame=}"

  def test_eps_timer_reset_aborts_on_steering_request(self):
    hca_mitigation = HCAMitigation(CCP, eps_timer_workaround=True)
    mitigation_start_calls = round(HCAMitigation.MLB_LOCKOUT_MITIGATION_START / DT_CTRL / CCP.STEER_STEP) + 1
    low_torque_calls = round(HCAMitigation.MLB_LOCKOUT_LOW_TORQUE_TIME / DT_CTRL / CCP.STEER_STEP) + 1

    pinned_output = 1
    assert pinned_output <= HCAMitigation.MLB_LOCKOUT_LOW_TORQUE

    for _ in range(mitigation_start_calls):
      apply_torque = hca_mitigation.update(CCP.STEER_MAX, CCP.STEER_MAX, CCP.STEER_MAX)
      assert apply_torque != 0, "reset must not trigger while the model is requesting high torque"

    for _ in range(low_torque_calls):
      apply_torque = hca_mitigation.update(pinned_output, pinned_output, 0)
    assert apply_torque == 0, "sustained low torque should zero the output to reset the EPS timer"

    apply_torque = hca_mitigation.update(pinned_output, 0, CCP.STEER_MAX)
    assert apply_torque == pinned_output, "reset must abort when the model commands real torque"

  def test_eps_timer_reset_completes(self):
    """The mitigation arms only after MLB_LOCKOUT_MITIGATION_START, and its reset clears the timer."""
    hca_mitigation = HCAMitigation(CCP, eps_timer_workaround=True)
    mitigation_start_calls = round(HCAMitigation.MLB_LOCKOUT_MITIGATION_START / DT_CTRL / CCP.STEER_STEP) + 1
    low_torque_calls = round(HCAMitigation.MLB_LOCKOUT_LOW_TORQUE_TIME / DT_CTRL / CCP.STEER_STEP) + 1
    reset_calls = round(CCP.STEER_TIME_RESET / DT_CTRL / CCP.STEER_STEP) + 1
    pinned_output = 1

    for _ in range(low_torque_calls * 2):
      apply_torque = hca_mitigation.update(pinned_output, 0, 0)
    assert apply_torque == pinned_output, "mitigation must not engage before MLB_LOCKOUT_MITIGATION_START"

    for _ in range(mitigation_start_calls):
      hca_mitigation.update(CCP.STEER_MAX, 0, CCP.STEER_MAX)
    for _ in range(low_torque_calls):
      apply_torque = hca_mitigation.update(pinned_output, 0, 0)
    assert apply_torque == 0, "sustained low torque should zero the output to reset the EPS timer"

    for _ in range(reset_calls):
      apply_torque = hca_mitigation.update(pinned_output, 0, 0)
    assert apply_torque == pinned_output, "EPS timer reset should complete and release steering"


class TestVolkswagenLkaHudControl(unittest.TestCase):
  """Lane-keep indicator lamp mapping for LDW_02 (green/yellow/off).

  Yellow wins over green so the cluster never shows green while the driver
  overrides steering or the car is at a standstill with latActive on.
  Frames are packed through the real DBCs (no mocks) so signal-layout
  regressions (bit positions, CHECKSUM/COUNTER autofill) are caught.
  """

  class _Hud:
    def __init__(self, left_visible=True, right_visible=True, left_depart=False, right_depart=False):
      self.leftLaneVisible = left_visible
      self.rightLaneVisible = right_visible
      self.leftLaneDepart = left_depart
      self.rightLaneDepart = right_depart

  @staticmethod
  def _pack(dbc_name, impl, lat_active, steering_pressed, v_ego, stock=None):
    packer = CANPacker(dbc_name)
    addr, dat, _ = impl(packer, 0, stock or {}, lat_active, steering_pressed, 0,
                        TestVolkswagenLkaHudControl._Hud(), v_ego=v_ego)
    msg = DBC(dbc_name).addr_to_msg[addr]

    def get(name):
      sig = msg.sigs[name]
      assert sig.is_little_endian, f"{name} decode helper only handles little-endian"
      raw = int.from_bytes(dat, "little") >> sig.lsb & ((1 << sig.size) - 1)
      return raw * sig.factor + sig.offset

    return dat, get

  def test_state_matrix(self):
    cases = [
      # (lat_active, steering_pressed, v_ego, expected_gruen, expected_gelb)
      (False, False, 10.0, 0, 0),   # off
      (False, False, 0.0,  0, 0),   # off at standstill
      (False, True,  10.0, 0, 0),   # off, driver pressing
      (True,  False, 10.0, 1, 0),   # active, driving -> green
      (True,  False, 0.0,  0, 1),   # active, stopped -> yellow
      (True,  True,  10.0, 0, 1),   # active, driver override -> yellow
      (True,  True,  0.0,  0, 1),   # both yellow conditions
      (True,  False, LANE_KEEP_STANDSTILL_M_S,        1, 0),  # at threshold -> green
      (True,  False, LANE_KEEP_STANDSTILL_M_S - 0.05, 0, 1),  # just under -> yellow
      (True,  False, LANE_KEEP_STANDSTILL_M_S + 0.05, 1, 0),  # just over -> green
    ]
    for dbc_name, impl in (("vw_mqb", mqb_create_lka_hud_control), ("vw_mlb", mlb_create_lka_hud_control)):
      for lat_act, sp, v, eg, ey in cases:
        with self.subTest(dbc=dbc_name, lat_active=lat_act, steering_pressed=sp, v_ego=v):
          _, get = self._pack(dbc_name, impl, lat_act, sp, v)
          self.assertEqual((get("LDW_Status_LED_gruen"), get("LDW_Status_LED_gelb")), (eg, ey))

  def test_default_v_ego_keeps_legacy_behavior(self):
    """Callers that don't pass v_ego get the pre-change mapping (green for plain lat_active)."""
    for dbc_name, impl in (("vw_mqb", mqb_create_lka_hud_control), ("vw_mlb", mlb_create_lka_hud_control)):
      packer = CANPacker(dbc_name)
      addr, dat, _ = impl(packer, 0, {}, True, False, 0, self._Hud())
      sig = DBC(dbc_name).addr_to_msg[addr].sigs["LDW_Status_LED_gruen"]
      raw = int.from_bytes(dat, "little") >> sig.lsb & ((1 << sig.size) - 1)
      self.assertEqual(raw, 1, f"{dbc_name}: no v_ego should still produce green")

  def test_stock_values_passthrough(self):
    """Seite/DLC/TLC/SW_Warnung must pass through from the stock camera frame."""
    stock = {"LDW_SW_Warnung_links": 1, "LDW_SW_Warnung_rechts": 0,
             "LDW_Seite_DLCTLC": 1, "LDW_DLC": 0.5, "LDW_TLC": 1.2}
    for dbc_name, impl in (("vw_mqb", mqb_create_lka_hud_control), ("vw_mlb", mlb_create_lka_hud_control)):
      with self.subTest(dbc=dbc_name):
        _, get = self._pack(dbc_name, impl, True, False, 10.0, stock=stock)
        self.assertEqual(get("LDW_SW_Warnung_links"), 1)
        self.assertEqual(get("LDW_Seite_DLCTLC"), 1)
        self.assertAlmostEqual(get("LDW_DLC"), 0.5, places=2)
        self.assertAlmostEqual(get("LDW_TLC"), 1.2, places=2)

  def test_mlb_checksum_counter_autofill(self):
    """vw_mlb LDW_02 frames must carry a computed checksum and a rolling counter."""
    packer = CANPacker("vw_mlb")
    addr1, dat1, _ = mlb_create_lka_hud_control(packer, 0, {}, True, False, 0, self._Hud(), v_ego=10.0)
    addr2, dat2, _ = mlb_create_lka_hud_control(packer, 0, {}, True, False, 0, self._Hud(), v_ego=10.0)
    self.assertEqual(addr1, 0x397)
    self.assertNotEqual(dat1[0], 0, "CHECKSUM byte must be computed")
    self.assertEqual((dat1[1] & 0x0F) + 1, dat2[1] & 0x0F, "COUNTER must roll")


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
