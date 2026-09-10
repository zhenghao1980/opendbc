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

  acc_01_values = {
    "ACC_Status_ACC": acc_control,
    "ACC_Sollbeschleunigung": accel if acc_enabled else 0,
    "ACC_zul_Regelabw_unten": 0.2,
    "ACC_zul_Regelabw_oben": 0.2,
    "ACC_neg_Sollbeschl_Grad": 4.0 if acc_enabled else 0,
    "ACC_pos_Sollbeschl_Grad": 4.0 if acc_enabled else 0,
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
    "ACC_Dynamik": 2,
    "ACC_Minimale_Bremsung": stopping,
  }
  commands.append(packer.make_can_msg("ACC_01", bus, acc_01_values))

  return commands


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  # TODO: happens to resemble the ACC control value for now, but extend this for init/gas override later
  return acc_control_value(main_switch_on, acc_faulted, long_active)


# B8 Kombi distance-bar display index (ACC_Abstandsindex) is a non-linear display index, not meters.
# Calibrated from 2478 stock radar<->vision lead pairs on B8PA (archived in acc_fusion/pairs_*.jsonl):
# (lead distance m, median abidx). Monotonic non-increasing; dense 17-60m region is high quality,
# far region (>70m) is noisy and saturates around 120.
_ABSTANDSINDEX_LUT = (
  (17.0, 668), (20.0, 616), (25.0, 505), (30.0, 472), (35.0, 442), (40.0, 433),
  (45.0, 417), (50.0, 400), (60.0, 388), (70.0, 267), (75.0, 243), (85.0, 145),
  (95.0, 120), (105.0, 120),
)


def _abstandsindex(lead_distance_m: float) -> int:
  """Map lead distance in meters to the B8 cluster's non-linear ACC_Abstandsindex display index."""
  if lead_distance_m <= _ABSTANDSINDEX_LUT[0][0]:
    return _ABSTANDSINDEX_LUT[0][1]
  for (d0, a0), (d1, a1) in zip(_ABSTANDSINDEX_LUT, _ABSTANDSINDEX_LUT[1:]):
    if lead_distance_m <= d1:
      return int(round(a0 + (a1 - a0) * (lead_distance_m - d0) / (d1 - d0)))
  return _ABSTANDSINDEX_LUT[-1][1]


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, hud_control, mlb_hud_text):

  acc_active = acc_hud_status in (3, 4)
  has_lead = acc_active and hud_control.leadVisible
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.04,
    # Stock J428 constants in all 5000 observed frames (active/passive/lead/no-lead):
    # Display_Prio=3, Typ_Tachokranz=1, Anzeige_Zeitluecke=0. OP must match these —
    # the Kombi drops the lead-car glyph when they deviate (distance bar still renders).
    "ACC_Display_Prio": 3,
    "ACC_Anzeige_Zeitluecke": 0,
    "ACC_Gesetzte_Zeitluecke": hud_control.leadDistanceBars, # TODO: Update openpilot charisma using stock rocker switch
    "ACC_Tachokranz": 1 if acc_active else 0,
    "ACC_Typ_Tachokranz": 1,
    "ACC_Relevantes_Objekt": 2 if hud_control.visualAlert > 0 else (1 if has_lead else 0),
    "ACC_Status_Prim_Anz": 2 if hud_control.visualAlert > 0 else (1 if acc_active else 0),
    "ACC_Akustik": 1 if hud_control.audibleAlert == 5 else 0, # Audible alert on OP warningImmediate
    # Stock J428 only draws the lead-car glyph when Abstandsindex carries a real distance index
    # (1023 = "road with green/red area" special display, 1022 = "grey road" special display)
    "ACC_Abstandsindex": _abstandsindex(lead_distance) if has_lead and lead_distance > 1.0 else (1023 if acc_active else 1022),
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
    seed ^= 0xAA

  return xor_checksum(address, sig, d, initial_value=seed)
