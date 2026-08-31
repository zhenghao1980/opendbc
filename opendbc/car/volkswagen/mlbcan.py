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


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control,
                           v_ego=0.0):
  # MLB-tuned LDW_02.
  #
  # We deliberately do NOT pass through the stock camera's DLC/TLC/Seite the
  # way the MQB version does: while OP is steering, the camera's departure
  # estimate does not describe OP's driving state, and mixing it with our
  # LED/Lernmodus bits gives the cluster an inconsistent picture. Conservative
  # "clear" geometry is synthesized instead. Real geometry from modelV2
  # laneLines is future work (needs controlsd-side integration); until then
  # the cluster will not render the red departure line.
  #
  # rlog evidence (route_b8pa, stock driving): DLC > 0 = clear of the line
  # (+0.4..+1.24 m in normal driving), DLC <= 0 = touching/crossing the line
  # (-0.49..-0.94 m during the 14 departure events). So the safe value is the
  # positive rail (+1.25 m); a negative value would read as "over the line".
  #
  # LDW_Seite_DLCTLC is a 1-bit field. Which value means which side could not
  # be derived from rlog and must be calibrated on-car. Mapping below is the
  # assumed default; flagged TODO for on-car verification.
  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",
      "LDW_SW_Warnung_rechts",
    ]}

  # Side of imminent departure (1-bit; mapping unverified, see header comment)
  if hud_control.leftLaneDepart and not hud_control.rightLaneDepart:
    values["LDW_Seite_DLCTLC"] = 0  # TODO(on-car): verify 0 == left on B8PA
  elif hud_control.rightLaneDepart and not hud_control.leftLaneDepart:
    values["LDW_Seite_DLCTLC"] = 1  # TODO(on-car): verify 1 == right on B8PA

  # Conservative geometry: far from either line, no imminent crossing, so the
  # cluster never draws a spurious red line. DBC ranges: DLC [-1.25,+1.25] m,
  # TLC [0,3.0] s.
  values["LDW_DLC"] = 1.25
  values["LDW_TLC"] = 3.0

  values.update({
    "LDW_Status_LED_gelb": 1 if lat_active and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if lat_active and not steering_pressed else 0,
    # Lernmodus: this cluster does not read these bits (rlog: stock sends them
    # constantly 0 and the display still works). Kept MQB-compatible.
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


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


def acc_hud_status_value(main_switch_on, acc_faulted, long_active, gas_pressed=False):
  # Status=4 = "background override": driver pressing gas pedal while longActive.
  # rlog evidence (route_b8pa, 60447 frames): stock car shows Status=4 + Prim=0
  # when driver takes over with throttle; OP without this fix left Prim=1 (green
  # ACC icon) which contradicts the stock cluster display.
  # This is the MLB-private implementation; the MQB/MEB/PQ version in
  # mqbcan.py is unchanged.
  if acc_faulted:
    return acc_control_value(main_switch_on, acc_faulted, long_active)
  if long_active and gas_pressed:
    return 4
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


def volkswagen_mlb_checksum(address: int, sig, d: bytearray) -> int:

  # LH_EPS_03, ACC_10, LH_EPS_02, ESP_08, HCA_01, LH_EPS_01
  if address in {0x9F, 0x117, 0x11D, 0x11E, 0x126, 0x32A}:
    return volkswagen_mqb_meb_checksum(address, sig, d)

  # XOR checksum is seeded with the CAN address high byte XOR low byte.
  seed = (address >> 8) ^ (address & 0xFF)
  if address in (0x100, 0x101): # ESP_01, ESP_02 special case
    seed ^= 0xAA

  return xor_checksum(address, sig, d, initial_value=seed)
