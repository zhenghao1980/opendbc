import unittest

from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from opendbc.car.volkswagen import mlbcan
from opendbc.car.volkswagen.mlbcan import acc_control_value, acc_hud_status_value, create_acc_hud_control, \
                                          create_lka_lamp_control, _abstandsindex, _ABIDX2D, _ABIDX2D_D_M, _ABIDX2D_V_KPH, \
                                          LKA_LAMP_OFF, LKA_LAMP_GREEN, LKA_LAMP_YELLOW


class _HudControl:
  """Minimal hudControl stand-in for create_acc_hud_control."""
  def __init__(self, lead_visible=True, lead_distance_bars=3, visual_alert=0, audible_alert=0):
    self.leadVisible = lead_visible
    self.leadDistanceBars = lead_distance_bars
    self.visualAlert = visual_alert
    self.audibleAlert = audible_alert


def _get_sig(dbc_name, addr, dat, name):
  msg = DBC(dbc_name).addr_to_msg[addr]
  sig = msg.sigs[name]
  assert sig.is_little_endian, f"{name}: decode helper only handles little-endian"
  raw = int.from_bytes(dat, "little") >> sig.lsb & ((1 << sig.size) - 1)
  return raw * sig.factor + sig.offset


class TestAccHudStatusValue(unittest.TestCase):
  """R-23 (1): ACC_02 Status state machine incl. MLB-private Status=4 gas override."""

  def test_base_status_mapping(self):
    self.assertEqual(acc_control_value(main_switch_on=False, acc_faulted=False, long_active=False), 0)
    self.assertEqual(acc_control_value(main_switch_on=True, acc_faulted=False, long_active=False), 2)
    self.assertEqual(acc_control_value(main_switch_on=True, acc_faulted=False, long_active=True), 3)
    self.assertEqual(acc_control_value(main_switch_on=True, acc_faulted=True, long_active=True), 6)

  def test_gas_override_status4(self):
    # Status=4 = "background override": driver on gas while longActive
    self.assertEqual(acc_hud_status_value(True, False, True, gas_pressed=True), 4)
    self.assertEqual(acc_hud_status_value(True, False, True, gas_pressed=False), 3)

  def test_gas_override_requires_long_active(self):
    # Gas without longitudinal must NOT produce 4
    self.assertEqual(acc_hud_status_value(True, False, False, gas_pressed=True), 2)
    self.assertEqual(acc_hud_status_value(False, False, False, gas_pressed=True), 0)

  def test_fault_dominates_gas_override(self):
    self.assertEqual(acc_hud_status_value(True, True, True, gas_pressed=True), 6)

  def test_default_gas_argument_is_false(self):
    # Backward-compatible signature: omitting gas_pressed keeps base mapping
    self.assertEqual(acc_hud_status_value(True, False, True), 3)


class TestAbstandsindexLut(unittest.TestCase):
  """2D (v_ego, distance) Abstandsindex LUT: anchors, bilinear interpolation, clamps."""

  def test_lut_anchor_points_exact(self):
    for i, v in enumerate(_ABIDX2D_V_KPH):
      for j, d in enumerate(_ABIDX2D_D_M):
        self.assertEqual(_abstandsindex(d, v), _ABIDX2D[i][j], f"anchor {v} kph / {d} m")

  def test_low_clamp_below_10m(self):
    # Grid starts at 10 m: clamped to the d=10 column (v=50 kph row -> 796)
    for d in (0.0, 5.0, 9.9, 10.0):
      self.assertEqual(_abstandsindex(d, 50.0), 796, f"{d} m")

  def test_high_clamp_above_115m(self):
    for d in (115.0, 150.0, 250.0):
      self.assertEqual(_abstandsindex(d, 50.0), 425, f"{d} m")

  def test_speed_clamp(self):
    # v below/above the 15-95 kph grid clamps to the edge rows (d=35 m)
    self.assertEqual(_abstandsindex(35.0, 0.0), _ABIDX2D[0][5])
    self.assertEqual(_abstandsindex(35.0, 200.0), _ABIDX2D[-1][5])

  def test_interpolation_distance_midpoint(self):
    # v=50 kph row, between d=30 (463) and d=35 (445): linear at 32.5 m
    self.assertEqual(_abstandsindex(32.5, 50.0), round(463 + (445 - 463) * 0.5))

  def test_interpolation_speed_midpoint(self):
    # d=50 m, between v=50 (425) and v=55 (399) rows at 52.5 kph
    self.assertEqual(_abstandsindex(50.0, 52.5), round(425 + (399 - 425) * 0.5))

  def test_monotonic_nonincreasing(self):
    for v in (15.0, 35.0, 50.0, 70.0, 95.0):
      prev = None
      for i in range(0, 2501):
        d = i * 0.1  # 0 .. 250 m
        a = _abstandsindex(d, v)
        if prev is not None:
          self.assertLessEqual(a, prev, f"non-monotonic at {v} kph / {d:.1f} m")
        prev = a


class TestAccHudControl(unittest.TestCase):
  """R-23 (4): ACC_02 display/announcement state machine (Display_Prio, display_armed,
  Abstandsindex special codes 1022/1023)."""

  def _pack(self, status, lead_distance=50.0, hud=None, announcing=False, display_armed=False,
            stock_relevant_obj=0, stock_abstandsindex=1023):
    packer = CANPacker("vw_mlb")
    hud = hud or _HudControl()
    return create_acc_hud_control(packer, 0, status, 100.0, lead_distance, hud, 0,
                                  announcing=announcing, display_armed=display_armed,
                                  stock_relevant_obj=stock_relevant_obj,
                                  stock_abstandsindex=stock_abstandsindex)

  def test_announcement_raises_display_prio(self):
    addr, dat, _ = self._pack(3, announcing=True)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Display_Prio"), 2)
    addr, dat, _ = self._pack(3, announcing=False)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Display_Prio"), 3)

  def test_announcement_ignored_when_not_displaying(self):
    # Status=0 (off): acc_display False, announcement must not raise prio
    addr, dat, _ = self._pack(0, announcing=True, display_armed=False)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Display_Prio"), 3)

  def test_display_armed_preframes_activate_display(self):
    # Stock J428 pre-frames: display fields arm ~0.1 s before Status flips to 3
    addr, dat, _ = self._pack(2, display_armed=True)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Tachokranz"), 1)
    addr, dat, _ = self._pack(2, display_armed=False)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Tachokranz"), 0)

  def test_abstandsindex_special_codes_no_lead(self):
    # 1023 = "road with green/red area" (displaying, no valid lead distance)
    addr, dat, _ = self._pack(3, hud=_HudControl(lead_visible=False))
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 1023)
    # 1022 = "grey road" (not displaying)
    addr, dat, _ = self._pack(0)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 1022)

  def test_abstandsindex_uses_lut_with_lead(self):
    # default v_ego_kph=50 row: d=40 -> 425, d=45 -> 425; flat at 42 m
    addr, dat, _ = self._pack(3, lead_distance=42.0)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 425)

  def test_abstandsindex_subliminal_lead_distance(self):
    # lead_distance <= 1.0 m is not a real lead: special code, not the clamp
    addr, dat, _ = self._pack(3, lead_distance=0.5)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 1023)

  def test_set_speed_invalid_sentinel(self):
    # vCruise > 250 must map to the OEM "invalid" sentinel 327.04
    packer = CANPacker("vw_mlb")
    addr, dat, _ = create_acc_hud_control(packer, 0, 2, 327.04, 50.0, _HudControl(), 0)
    self.assertAlmostEqual(_get_sig("vw_mlb", addr, dat, "ACC_Wunschgeschw_02"), 327.04, places=1)
    addr, dat, _ = create_acc_hud_control(packer, 0, 2, 300.0, 50.0, _HudControl(), 0)
    self.assertAlmostEqual(_get_sig("vw_mlb", addr, dat, "ACC_Wunschgeschw_02"), 327.04, places=1)


class TestLeadDisplayArbitration(unittest.TestCase):
  """Lead-car display arbitration: stock J428 radar verdict (0/1/2) and native
  Abstandsindex win over the OP vision fit by default; OP only fills in a lead
  the radar does not see. Native index passes through unfiltered (1022/1023
  special displays included)."""

  def _pack(self, status=3, lead_distance=50.0, hud=None, stock_relevant_obj=0, stock_abstandsindex=1023):
    packer = CANPacker("vw_mlb")
    hud = hud or _HudControl()
    return create_acc_hud_control(packer, 0, status, 100.0, lead_distance, hud, 0,
                                  stock_relevant_obj=stock_relevant_obj,
                                  stock_abstandsindex=stock_abstandsindex)

  def test_radar_green_wins_tie_and_passthrough(self):
    # radar=1, OP also sees a lead: display follows the radar, native index raw
    addr, dat, _ = self._pack(hud=_HudControl(lead_visible=True),
                              stock_relevant_obj=1, stock_abstandsindex=300)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Relevantes_Objekt"), 1)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 300)

  def test_radar_red_car_always_wins(self):
    # OP can never downgrade a radar red car (2) to green
    addr, dat, _ = self._pack(hud=_HudControl(lead_visible=False),
                              stock_relevant_obj=2, stock_abstandsindex=200)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Relevantes_Objekt"), 2)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 200)

  def test_op_only_lead_uses_fit(self):
    # radar=0, OP=1: legacy fitted index path
    addr, dat, _ = self._pack(lead_distance=42.0, hud=_HudControl(lead_visible=True),
                              stock_relevant_obj=0)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Relevantes_Objekt"), 1)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 425)  # 2D LUT @ (42 m, 50 kph default)

  def test_op_only_lead_subliminal_distance(self):
    # OP lead but distance <= 1.0 m: special display, not the LUT low clamp
    addr, dat, _ = self._pack(lead_distance=0.5, hud=_HudControl(lead_visible=True),
                              stock_relevant_obj=0)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Relevantes_Objekt"), 1)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 1023)

  def test_radar_special_index_not_filtered(self):
    # radar=1 carrying 1023 (stock "ACC active" presentation): passthrough
    addr, dat, _ = self._pack(hud=_HudControl(lead_visible=False),
                              stock_relevant_obj=1, stock_abstandsindex=1023)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Relevantes_Objekt"), 1)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 1023)

  def test_visual_alert_red_car_preserved(self):
    # FCW visualAlert still forces the red car with no radar/OP lead
    addr, dat, _ = self._pack(hud=_HudControl(lead_visible=False, visual_alert=1),
                              stock_relevant_obj=0)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Relevantes_Objekt"), 2)

  def test_radar_lead_gated_by_acc_active(self):
    # ACC off: radar verdict is not displayed, grey road special display
    addr, dat, _ = self._pack(status=0, hud=_HudControl(lead_visible=False),
                              stock_relevant_obj=1, stock_abstandsindex=300)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Relevantes_Objekt"), 0)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 1022)

  def test_defaults_preserve_legacy_op_only_behavior(self):
    # No stock args (e.g. radar frames not wired): identical to old behavior
    addr, dat, _ = self._pack(lead_distance=42.0)  # _HudControl default: lead visible
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Relevantes_Objekt"), 1)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 425)  # 2D LUT @ (42 m, 50 kph default)
    addr, dat, _ = self._pack(lead_distance=42.0, hud=_HudControl(lead_visible=False))
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Relevantes_Objekt"), 0)
    self.assertEqual(_get_sig("vw_mlb", addr, dat, "ACC_Abstandsindex"), 1023)


class TestLkaLampControl(unittest.TestCase):
  """R-23 (3): 0x30A LKA_LAMP byte1/byte2/byte3 combinations."""

  def _pack(self, lat_active, steering_pressed=False, v_ego=10.0, departing=False, lat_enabled=False):
    packer = CANPacker("vw_mlb")
    return create_lka_lamp_control(packer, 0, lat_active, steering_pressed, v_ego=v_ego,
                                   departing=departing, lat_enabled=lat_enabled)

  def test_byte2_state_mapping(self):
    cases = [
      # (lat_active, steering_pressed, v_ego, want_byte2)
      (False, False, 10.0, LKA_LAMP_OFF),
      (True,  False, 10.0, LKA_LAMP_GREEN),
      (True,  True,  10.0, LKA_LAMP_YELLOW),   # driver override -> yellow wins
      (True,  False, 0.0,  LKA_LAMP_YELLOW),   # standstill -> yellow wins
      (True,  False, None, LKA_LAMP_GREEN),    # unknown speed -> legacy green
    ]
    for lat, sp, v, want in cases:
      with self.subTest(lat_active=lat, steering_pressed=sp, v_ego=v):
        _, dat, _ = self._pack(lat, sp, v)
        self.assertEqual(dat[2], want)

  def test_lanes_recognized_only_when_green(self):
    _, dat, _ = self._pack(True, v_ego=10.0)
    self.assertEqual(dat[3] & 0x01, 0x01)  # green -> lanes recognized
    _, dat, _ = self._pack(True, v_ego=0.0)
    self.assertEqual(dat[3] & 0x01, 0x00)  # yellow -> hollow lines
    _, dat, _ = self._pack(False, v_ego=10.0)
    self.assertEqual(dat[3], 0x00)

  def test_departure_warning_bits(self):
    _, dat, _ = self._pack(True, v_ego=10.0, departing=True)
    self.assertEqual(dat[3] & 0x02, 0x02)  # byte3 bit1
    self.assertEqual(dat[1] & 0x40, 0x40)  # byte1 bit6
    # departure gated off when lateral inactive (stock: warnings only when active)
    _, dat, _ = self._pack(False, v_ego=10.0, departing=True)
    self.assertEqual(dat[3] & 0x02, 0x00)
    self.assertEqual(dat[1] & 0x40, 0x00)

  def test_byte7_constant_and_spare_bytes_zero(self):
    _, dat, _ = self._pack(True, v_ego=10.0, departing=True)
    self.assertEqual(dat[7], 0x80)
    for i in (0, 4, 5, 6):
      self.assertEqual(dat[i], 0, f"byte{i} must stay zero")

  def test_lat_enabled_keeps_lamp_alive_at_standstill(self):
    # upstream gates latActive off at standstill (steerAtStandstill=False on MLB);
    # the stock camera keeps yellow on there -> follow enabled state, not active
    _, dat, _ = self._pack(False, v_ego=0.0, lat_enabled=True)
    self.assertEqual(dat[2], LKA_LAMP_YELLOW)
    _, dat, _ = self._pack(False, v_ego=0.0, lat_enabled=False)
    self.assertEqual(dat[2], LKA_LAMP_OFF)


if __name__ == "__main__":
  unittest.main()
