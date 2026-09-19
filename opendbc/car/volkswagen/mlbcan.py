from bisect import bisect_right

from opendbc.car.crc import CRC8H2F
from opendbc.car.volkswagen.mqbcan import volkswagen_mqb_meb_checksum, xor_checksum

# TODO: Parameterize the hca control type (5 vs 7) and consolidate with MQB (and PQ?)
def create_steering_control(packer, bus, apply_steer, lkas_enabled, hca_mode=7):
  values = {
    "HCA_01_Status_HCA": hca_mode if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_01_Vib_Freq": 18,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
    "EA_ACC_Wunschgeschwindigkeit": 327.36,
  }
  return packer.make_can_msg("HCA_01", bus, values)


# v_ego standstill threshold (m/s) for the yellow lane-keep indicator.
# At ~1.8 km/h (0.5 m/s) the car is effectively stopped (traffic light,
# stop-and-go creep) while openpilot lateral may still be active; the cluster
# should show yellow ("engaged but not steering"), not green.
# Threshold is intentionally above zero so the lamp doesn't bounce green/yellow
# while creeping.
LANE_KEEP_STANDSTILL_M_S = 0.5


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control,
                           v_ego=None):
  # MLB-native version of mqbcan.create_lka_hud_control (kept here so mqbcan.py
  # stays byte-identical to upstream). Adds v_ego-aware yellow-lamp precedence:
  # yellow wins over green so the cluster never shows green while the driver
  # overrides steering or the car is at a standstill with latActive on.
  # v_ego=None means "not supplied": keep the legacy mapping (no standstill yellow).
  standstill = v_ego is not None and v_ego < LANE_KEEP_STANDSTILL_M_S
  yellow = lat_active and (steering_pressed or standstill)
  green = lat_active and not yellow

  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",   # Blind spot in warning mode on left side due to lane departure
      "LDW_SW_Warnung_rechts",  # Blind spot in warning mode on right side due to lane departure
      "LDW_Seite_DLCTLC",       # Direction of most likely lane departure (left or right)
      "LDW_DLC",                # Lane departure, distance to line crossing
      "LDW_TLC",                # Lane departure, time to line crossing
    ]}

  values.update({
    "LDW_Status_LED_gelb": 1 if yellow else 0,
    "LDW_Status_LED_gruen": 1 if green else 0,
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


# B8 cluster lane-keep lamp is driven by camera message 0x30A byte2, NOT by the
# LDW_02 (0x397) LED bits (those are D4/C7-era and ignored by the B8 Kombi —
# rlog-verified: OP's LDW_02 LED green frames never changed the lamp, while the
# camera's 0x30A byte2 tracked it exactly). byte2: 0x00=off, 0x88=yellow,
# 0x10=green (lanes-ok bit; stock sets it only above ~60 km/h with clear lanes).
# No checksum, no counter; byte7 is always 0x80.
#
# FIS lane-line graphic (2026-09-05 stock rlog analysis, 62 highway segments):
# LDW_02 carries no line state on B8 (all display fields constant 0); 0x30A is
# the only message tracking lane recognition. byte3 bit0 = lanes recognized
# (stock: set exactly when byte2 green bit is set, 72k frames, no exception);
# on-car: yellow (byte2=0x88, byte3=0x00) shows hollow lines, so byte3 bit0 is
# the hollow->solid switch. byte3 bit1 + byte1 bit6 = lane-departure warning
# (red line): 30 stock episodes, all within 0.01-0.7 m of the line per LDW_02 DLC.
LKA_LAMP_OFF = 0x00
LKA_LAMP_GREEN = 0x10
LKA_LAMP_YELLOW = 0x88
LKA_LAMP_LANES_RECOGNIZED = 0x01
LKA_LAMP_LANES_DEPARTURE = 0x02
LKA_LAMP_WARN_ACTIVE = 0x40


def create_lka_lamp_control(packer, bus, lat_active, steering_pressed, v_ego=None, departing=False, lat_enabled=False):
  # Yellow-wins precedence: driver override, standstill, or "enabled but not
  # actively steering" -> yellow; plain latActive -> green; otherwise off.
  # lat_enabled keeps the lamp alive at standstill: upstream controlsd gates
  # latActive off at standstill (steerAtStandstill=False on MLB), but the stock
  # camera keeps the yellow lamp on there, so the display state must follow the
  # enabled/wanted state, not the steering-active state.
  standstill = v_ego is not None and v_ego < LANE_KEEP_STANDSTILL_M_S
  lateral_on = lat_active or lat_enabled
  yellow = lateral_on and (steering_pressed or standstill or not lat_active)
  green = lat_active and not yellow
  byte2 = LKA_LAMP_YELLOW if yellow else (LKA_LAMP_GREEN if green else LKA_LAMP_OFF)
  # Departure warning bits only make sense while the system is active (stock
  # warning episodes all occurred in active states); keep them off otherwise.
  dep = departing and lat_active
  byte3 = (LKA_LAMP_LANES_RECOGNIZED if green else 0x00) | (LKA_LAMP_LANES_DEPARTURE if dep else 0x00)
  byte1 = LKA_LAMP_WARN_ACTIVE if dep else 0x00
  return packer.make_can_msg("LKA_LAMP", bus, {
    "LKA_Lamp_Warn": byte1,
    "LKA_Lamp_State": byte2,
    "LKA_Lamp_Lanes": byte3,
    "LKA_Lamp_Const": 0x80,
  })


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "LS_Hauptschalter",
    "LS_Typ_Hauptschalter",
    "LS_Codierung",
    "LS_Tip_Stufe_2",
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": cancel,
    "LS_Tip_Wiederaufnahme": resume,
  })

  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    acc_control = 6
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  commands = []

  # OEM-style jerk envelope, reverse-engineered from factory J428 logs (65 min
  # highway + 59 min urban, ~370k frames, oem_log_stats*.py):
  #   not regulating          -> both grads 0 (matches st2 share exactly)
  #   regulating, brake dir   -> neg_grad constant 3.5 across the full
  #                              -3.0..+3.0 accel range (urban deep-brake
  #                              samples confirm; safety: brake response is
  #                              never self-limited)
  #   regulating, release dir -> pos_grad scales with brake depth:
  #                              urban fit |a|+0.6 (highway fit |a|+0.25),
  #                              clipped to [0.6, 3.2]. Shallower = softer.
  # 非调节→双 0 对应 J428 standby 语义,OP 侧以 acc_enabled 为调节判据
  # (OP 作为虚拟 J428 发送 ACC_01,acc_enabled 即自身的调节状态)。
  if acc_enabled:
    neg_grad = 3.5
    pos_grad = min(max(abs(accel) + 0.6, 0.6), 3.2)
  else:
    neg_grad = 0.0
    pos_grad = 0.0

  acc_01_values = {
    "ACC_Status_ACC": acc_control,
    "ACC_Sollbeschleunigung": accel if acc_enabled else 0,
    "ACC_zul_Regelabw_unten": 0.2,
    "ACC_zul_Regelabw_oben": 0.2,
    "ACC_neg_Sollbeschl_Grad": neg_grad,
    "ACC_pos_Sollbeschl_Grad": pos_grad,
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
    "ACC_Dynamik": 2,
    "ACC_Minimale_Bremsung": stopping,
  }
  commands.append(packer.make_can_msg("ACC_01", bus, acc_01_values))

  return commands


def acc_hud_status_value(main_switch_on, acc_faulted, long_active, gas_pressed=False):
  # Status=4 = "background override": driver pressing gas pedal while longActive.
  # rlog evidence (route_b8pa): stock car shows Status=4 + Prim=0 when the driver
  # takes over with throttle; without this, Prim stays 1 (green ACC icon), which
  # contradicts the stock cluster display.
  # This is the MLB-private implementation; the MQB/MEB/PQ version in mqbcan.py is unchanged.
  # (restored from ba57a6cc; lost in the 2026-09-11 zip export)
  if acc_faulted:
    return acc_control_value(main_switch_on, acc_faulted, long_active)
  if long_active and gas_pressed:
    return 4
  return acc_control_value(main_switch_on, acc_faulted, long_active)


# B8 Kombi distance-bar display index (ACC_Abstandsindex) is a non-linear display index, not meters.
# 2D table over (ego speed, lead distance): the stock J428 index is a composite of both
# (approx. headway-like), which a distance-only LUT cannot reproduce (CV R2 0.669 -> 0.884,
# MAE 47.7 -> 20.2 index units on 82,373 paired frames).
# Sources: calib-20260916 (61 segs, gap 1/2) + rawdata (68 segs, gap 1) + op-acc02 rlog34;
# ACC_02(0x30C, bus2) abidx paired with modelV2 lead x and ESP wheel speed.
# Cell = median abidx in d+-2.5m / v+-2.5kph (n>=30); holes interpolated per speed row;
# enforced non-increasing in d. Bilinear interpolation between anchors; clamped at edges.
# Zeitluecke setting shown to NOT affect abidx (median residual ~0 across gap 1/2).
_ABIDX2D_V_KPH = (15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95)
_ABIDX2D_D_M = (10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115)
_ABIDX2D = (
  (594, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508),  # 15 kph
  (594, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508, 508),  # 20 kph
  (755, 512, 479, 439, 398, 358, 317, 277, 277, 234, 218, 170, 157, 144, 123, 99, 99, 99, 99, 99, 99, 99),        # 25 kph
  (753, 513, 497, 454, 438, 416, 393, 371, 352, 352, 352, 300, 177, 54, 54, 54, 54, 54, 54, 54, 54, 54),         # 30 kph
  (785, 669, 505, 449, 432, 420, 404, 404, 404, 404, 404, 404, 404, 404, 404, 404, 404, 404, 404, 404, 404, 404),  # 35 kph
  (828, 662, 496, 477, 434, 431, 412, 332, 301, 301, 301, 301, 301, 301, 301, 301, 301, 301, 301, 301, 301, 301),  # 40 kph
  (686, 686, 511, 484, 449, 426, 409, 409, 409, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282),  # 45 kph
  (796, 796, 613, 495, 463, 445, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425),  # 50 kph
  (816, 816, 622, 510, 476, 451, 430, 408, 399, 384, 377, 372, 372, 372, 372, 372, 372, 372, 372, 372, 372, 372),  # 55 kph
  (802, 802, 694, 528, 481, 452, 425, 419, 399, 399, 335, 326, 263, 263, 263, 263, 251, 251, 232, 213, 213, 213),  # 60 kph
  (788, 788, 653, 525, 484, 465, 449, 408, 399, 387, 382, 338, 240, 240, 240, 240, 240, 240, 240, 187, 187, 187),  # 65 kph
  (760, 760, 760, 555, 505, 463, 453, 424, 390, 378, 374, 370, 370, 270, 171, 171, 162, 155, 142, 122, 122, 122),  # 70 kph
  (770, 770, 770, 626, 510, 500, 452, 417, 395, 381, 381, 341, 330, 317, 312, 297, 282, 282, 266, 266, 266, 266),  # 75 kph
  (631, 631, 631, 631, 509, 495, 474, 433, 417, 368, 356, 331, 296, 266, 266, 188, 146, 103, 62, 62, 62, 62),     # 80 kph
  (508, 508, 508, 508, 508, 490, 486, 450, 422, 398, 385, 350, 324, 296, 295, 277, 260, 244, 114, 84, 68, 68),    # 85 kph
  (422, 422, 422, 422, 422, 422, 422, 422, 422, 394, 360, 323, 290, 259, 238, 226, 222, 210, 156, 146, 126, 126), # 90 kph
  (422, 422, 422, 422, 422, 422, 422, 422, 422, 394, 360, 323, 290, 259, 238, 226, 222, 210, 156, 146, 126, 126), # 95 kph
)


def _abstandsindex_row_interp(row, d: float) -> float:
  j = max(0, min(bisect_right(_ABIDX2D_D_M, d) - 1, len(_ABIDX2D_D_M) - 2))
  d0, d1 = _ABIDX2D_D_M[j], _ABIDX2D_D_M[j + 1]
  return row[j] + (row[j + 1] - row[j]) * (d - d0) / (d1 - d0)


def _abstandsindex(lead_distance_m: float, v_ego_kph: float = 50.0) -> int:
  """Map (lead distance, ego speed) to the B8 cluster's non-linear ACC_Abstandsindex display index.

  Bilinear lookup over _ABIDX2D; inputs clamped to the calibrated grid
  (10-115 m, 15-95 kph). v_ego_kph defaults to a mid-grid neutral speed."""
  v = min(max(v_ego_kph, _ABIDX2D_V_KPH[0]), _ABIDX2D_V_KPH[-1])
  d = min(max(lead_distance_m, _ABIDX2D_D_M[0]), _ABIDX2D_D_M[-1])
  i = max(0, min(bisect_right(_ABIDX2D_V_KPH, v) - 1, len(_ABIDX2D_V_KPH) - 2))
  v0, v1 = _ABIDX2D_V_KPH[i], _ABIDX2D_V_KPH[i + 1]
  a_lo = _abstandsindex_row_interp(_ABIDX2D[i], d)
  a_hi = _abstandsindex_row_interp(_ABIDX2D[i + 1], d)
  return int(round(a_lo + (a_hi - a_lo) * (v - v0) / (v1 - v0)))


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, hud_control, mlb_hud_text, announcing=False,
                           display_armed=False, stock_relevant_obj=0, stock_abstandsindex=1023, v_ego_kph=50.0):

  acc_active = acc_hud_status in (3, 4)
  # display_armed: stock J428 pre-frames — display fields switch to the active
  # presentation ~0.1s BEFORE ACC_Status flips to 3 (sent the moment the driver
  # presses SET/RES). The Kombi requires this arm->activate sequence to (re)draw
  # the lead-car graphic after a cancel.
  acc_display = acc_active or display_armed

  # Lead-car display arbitration (B8PA): the stock J428 radar keeps publishing
  # its own object verdict (ACC_Relevantes_Objekt 0/1/2 = none/green/red) and
  # native distance index on the radar-side bus even while OP regulates. The
  # display follows the radar by default; OP vision only adds a lead the radar
  # does not see (op 0/1 can never downgrade a radar red car to green).
  op_lead = 1 if hud_control.leadVisible else 0
  radar_lead = int(stock_relevant_obj)
  lead_display = max(radar_lead, op_lead, 2 if hud_control.visualAlert > 0 else 0)
  has_lead = acc_active and lead_display > 0

  if has_lead:
    if radar_lead >= op_lead:
      # Radar (or tie): passthrough the J428 native index untouched, including
      # 1022/1023 special displays — that IS the stock presentation.
      abstandsindex = int(stock_abstandsindex)
    else:
      # OP-only lead: fitted index from vision distance + ego speed (2D LUT path).
      abstandsindex = _abstandsindex(lead_distance, v_ego_kph) if lead_distance > 1.0 else (1023 if acc_display else 1022)
  else:
    abstandsindex = 1023 if acc_display else 1022

  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    # Stock J428 keeps the stored set speed VALID in standby after a cancel
    # (B8PA rlog route 0000000a: standby frames hold w=69.76); it is only
    # invalid before the first set. OP matches this by always sending it.
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.04,
    # Stock J428 display-priority state machine (B8PA rlog route 0000000a,
    # full cancel->resume ground truth):
    #   steady/standby:  Display_Prio=3, Anzeige_Zeitluecke=0
    #   announcement (~2s after activation, speed set, or gap change):
    #                    Display_Prio=2, Texte_Primaeranz=announcement text
    # The Kombi (re)draws the ACC lead-car/road graphic when it receives an
    # announcement. OP previously announced only on set-speed CHANGES, so the
    # graphic appeared at first activation (327->valid speed change) but a
    # resume with unchanged speed announced nothing and was never redrawn.
    "ACC_Display_Prio": 2 if (acc_display and announcing) else 3,
    "ACC_Anzeige_Zeitluecke": 0,
    "ACC_Gesetzte_Zeitluecke": hud_control.leadDistanceBars, # TODO: Update openpilot charisma using stock rocker switch
    "ACC_Tachokranz": 1 if acc_display else 0,
    "ACC_Typ_Tachokranz": 1,
    "ACC_Relevantes_Objekt": lead_display if (acc_active or hud_control.visualAlert > 0) else 0,
    # Stock holds Status_Prim_Anz=1 essentially the whole CONTROLLING period
    # (route 0000000a/0000000f steady state) but drops it to 0 during gas
    # override (Status=4, route 0000000f t=872.37).
    "ACC_Status_Prim_Anz": 2 if hud_control.visualAlert > 0 else (1 if acc_hud_status == 3 else 0),
    "ACC_Akustik": 1 if hud_control.audibleAlert == 5 else 0, # Audible alert on OP warningImmediate
    # Stock J428 only draws the lead-car glyph when Abstandsindex carries a real distance index
    # (1023 = "road with green/red area" special display, 1022 = "grey road" special display)
    "ACC_Abstandsindex": abstandsindex,
    "ACC_Texte_Primaeranz": mlb_hud_text,
  }

  return packer.make_can_msg("ACC_02", bus, values)


# MLB-only CRC8H2F initial values, keyed by message address, one per counter
# value. Kept here (not in mqbcan's VOLKSWAGEN_MQB_MEB_CONSTANTS) so mqbcan.py
# stays byte-identical to upstream.
MLB_CRC8_CONSTANTS: dict[int, list[int]] = {
  0x11D: [0x1C] * 16,  # LH_EPS_02
  0x11E: [0xD2] * 16,  # ESP_08
  0x32A: [0x29] * 16,  # LH_EPS_01
}


def volkswagen_mlb_checksum(address: int, sig, d: bytearray) -> int:

  # LH_EPS_03, ACC_10, HCA_01 use the shared MQB/MEB constant table
  if address in {0x9F, 0x117, 0x126}:
    return volkswagen_mqb_meb_checksum(address, sig, d)

  # LH_EPS_02, ESP_08, LH_EPS_01: same CRC8H2F algorithm, MLB-local constants
  if address in MLB_CRC8_CONSTANTS:
    crc = 0xFF
    for i in range(1, len(d)):
      crc ^= d[i]
      crc = CRC8H2F[crc]
    crc ^= MLB_CRC8_CONSTANTS[address][d[1] & 0x0F]
    crc = CRC8H2F[crc]
    return crc ^ 0xFF

  # XOR checksum is seeded with the CAN address high byte XOR low byte.
  seed = (address >> 8) ^ (address & 0xFF)
  if address in (0x100, 0x101): # ESP_01, ESP_02 special case
    # TODO(R-30): cite the stock rlog (route_id/segment/frame count) that
    # established the 0xAA seed for ESP_01/ESP_02
    seed ^= 0xAA

  return xor_checksum(address, sig, d, initial_value=seed)
