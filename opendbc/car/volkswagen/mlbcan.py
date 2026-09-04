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
LKA_LAMP_OFF = 0x00
LKA_LAMP_GREEN = 0x10
LKA_LAMP_YELLOW = 0x88


def create_lka_lamp_control(packer, bus, lat_active, steering_pressed, v_ego=None):
  # Same yellow-wins precedence as create_lka_hud_control: driver override or
  # standstill -> yellow; plain latActive -> green; otherwise off.
  standstill = v_ego is not None and v_ego < LANE_KEEP_STANDSTILL_M_S
  yellow = lat_active and (steering_pressed or standstill)
  green = lat_active and not yellow
  byte2 = LKA_LAMP_YELLOW if yellow else (LKA_LAMP_GREEN if green else LKA_LAMP_OFF)
  return packer.make_can_msg("LKA_LAMP", bus, {
    "LKA_Lamp_State": byte2,
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


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, hud_control, mlb_hud_text):

  acc_active = acc_hud_status in (3, 4)
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.04,
    "ACC_Display_Prio": 0,
    "ACC_Anzeige_Zeitluecke": 1 if acc_active else 0,
    "ACC_Gesetzte_Zeitluecke": hud_control.leadDistanceBars, # TODO: Update openpilot charisma using stock rocker switch
    "ACC_Tachokranz": 1 if acc_active else 0,
    "ACC_Relevantes_Objekt": 2 if hud_control.visualAlert > 0 else (1 if acc_active and hud_control.leadVisible else 0),
    "ACC_Status_Prim_Anz": 2 if hud_control.visualAlert > 0 else (1 if acc_active else 0),
    "ACC_Akustik": 1 if hud_control.audibleAlert == 5 else 0, # Audible alert on OP warningImmediate
    "ACC_Abstandsindex": 1023 if acc_active else 1022,
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
