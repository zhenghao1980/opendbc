import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs

ButtonType = structs.CarState.ButtonEvent.Type
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mebcan, mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class HCAMitigation:
  """
  Manages HCA fault mitigations for VW/Audi EPS racks:
    * Reduces torque by 1 for a single frame after commanding the same torque value for too long
    * For MLB racks: opportunistically disables HCA during low-torque periods before the 6-minute EPS lockout
  """

  MLB_LOCKOUT_MITIGATION_START = 240. # EPS engaged time before attempting opportunistic reset (sec)
  MLB_LOCKOUT_LOW_TORQUE = 60         # Desired torque must be less than this before and during reset (centi-Nm)
  MLB_LOCKOUT_LOW_TORQUE_TIME = 0.5   # How long to observe low torque for before starting the reset (sec)

  def __init__(self, CCP, eps_timer_workaround=False):
    self._max_same_torque_frames = CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP)
    self._same_torque_frames = 0

    self._eps_timer_workaround = eps_timer_workaround
    if eps_timer_workaround:
      self._steer_step = CCP.STEER_STEP
      self._hca_active_frames = 0
      self._hca_inactive_frames = 0
      self._low_torque_frames = 0
      self._frames_for_mitigation_start = self.MLB_LOCKOUT_MITIGATION_START / DT_CTRL
      self._frames_for_low_torque = self.MLB_LOCKOUT_LOW_TORQUE_TIME / DT_CTRL
      self._frames_for_reset = CCP.STEER_TIME_RESET / DT_CTRL

  def update(self, apply_torque, apply_torque_last, desired_torque):
    if apply_torque != 0 and apply_torque_last == apply_torque:
      self._same_torque_frames += 1
      if self._same_torque_frames > self._max_same_torque_frames:
        apply_torque -= (1, -1)[apply_torque < 0]
        self._same_torque_frames = 0
    else:
      self._same_torque_frames = 0

    # MLB steering racks have a 6min max engagement. After that time it will return status = 'rejected' for ~2.0s and not execute torque requests. After the
    # ~2.0s lockout period it will return to accepting torque requests by itself. This max engagement timer can also be reset by disabling control for ~1.1s.
    # Attempt to mitigate this by opportunistically disabling HCA during periods of low torque desired.
    if self._eps_timer_workaround:
      self._hca_active_frames += self._steer_step

      if (self._hca_active_frames >= self._frames_for_mitigation_start
          and abs(desired_torque) <= self.MLB_LOCKOUT_LOW_TORQUE):
        self._low_torque_frames += self._steer_step
      else:
        self._low_torque_frames = 0

      # Only disable HCA if desired torque is <=0.6Nm for 0.5 seconds continuously. A desired torque > 0.6Nm will also abort any in progress reset.
      if self._low_torque_frames >= self._frames_for_low_torque:
        apply_torque = 0

      if apply_torque == 0:
        self._hca_inactive_frames += self._steer_step
        if self._hca_inactive_frames >= self._frames_for_reset:
          self._hca_active_frames = 0
      else:
        self._hca_inactive_frames = 0

    return apply_torque


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    if CP.flags & VolkswagenFlags.MEB:
      self.meb_long_state = mebcan.MebLongStateMachine(self.CP, self.CCP)

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.apply_curvature_last = 0.
    self.steering_power_last = 0
    self.accel_last = 0.
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.gra_acc_counter_last = None
    self.hca_mitigation = HCAMitigation(self.CCP, eps_timer_workaround=bool(CP.flags & VolkswagenFlags.MLB))
    # B8PA 实测原厂 ALA 用 HCA Status 5 (oscar 的 Q5 用 7)
    self.hca_mode = 5 if CP.carFingerprint == "AUDI_A4_B8PA" else 7

    self.last_set_speed = 0
    self.last_lead_distance_bars = 0
    self.mlb_hud_text = 0
    self.texte_timer = 0
    self.last_long_active = False
    self.gra_arm_frame = -1000
    self.acc_gas_override = False

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if self.CP.flags & VolkswagenFlags.MEB:
        # Logic to avoid HCA refused state:
        #   * steering power as counter and near zero before OP lane assist deactivation
        # MEB rack can be used continuously without time limits
        # maximum real steering angle change ~ 120-130 deg/s

        if CC.latActive:
          hca_enabled = True
          # compensate the gap between measured and current curvature
          apply_curvature = actuators.curvature + (CS.curvature_meas - CC.currentCurvature)
          apply_curvature = self.CCP.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw, CS.curvature_meas,
                                                                   CC.latActive, self.CCP.STEER_STEP)

          min_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
          max_power = min(self.steering_power_last + self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MAX)
          target_power_driver = int(np.interp(abs(CS.out.steeringTorque), [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX],
                                                                          [self.CCP.STEERING_POWER_MAX, self.CCP.STEERING_POWER_MIN]))
          target_power = int(np.interp(CS.out.vEgo, [0., 0.5], [self.CCP.STEERING_POWER_MIN, target_power_driver]))
          steering_power = min(max(target_power, min_power), max_power)

        else:
          if self.steering_power_last > 0:  # keep HCA alive until steering power has reduced to zero
            hca_enabled = True
            apply_curvature = float(np.clip(CS.curvature_meas, -self.CCP.CURVATURE_MAX, self.CCP.CURVATURE_MAX))
            steering_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, 0)
          else:
            hca_enabled = False
            apply_curvature = 0.  # inactive curvature
            steering_power = 0

        can_sends.append(mebcan.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power))
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power

      else:
        new_torque = 0
        if CC.latActive:
          new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

        apply_torque = self.hca_mitigation.update(apply_torque, self.apply_torque_last, new_torque)
        hca_enabled = apply_torque != 0
        self.apply_torque_last = apply_torque
        if self.CP.flags & VolkswagenFlags.MLB:
          can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled,
                                                            hca_mode=self.hca_mode))
        else:
          can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # Emergency Assist intervention
    if self.CP.flags & VolkswagenFlags.MEB and self.CP.flags & VolkswagenFlags.STOCK_KLR_PRESENT:
      # send capacitive steering wheel hands-on message to keep ACC resume active and control Emergency Assist
      # MEB Emergency Assist brake jerks after 30s of continued hands-off time.
      # We send the stock wheeltouch message to start the stock DM timer when openpilot latches the critical driver monitoring alert
      if self.frame % self.CCP.KLR_01_STEP == 0:
        lat_active = CC.latActive and not CC.driverMonitoringEscalation
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.cam, lat_active, CS.klr_stock_values))
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.pt, lat_active, CS.klr_stock_values))

    # **** Acceleration Controls ******************************************** #

    if self.CP.flags & VolkswagenFlags.MLB:
      # Gas-override display latch: controlsd re-raises CC.longActive a frame or
      # two AFTER gasPressed clears, which produced a 1-frame ACC_01 status=2
      # (standby) blip at override exit (route 00000019 t=577.07/583.50). The
      # Kombi latches that blip as "ACC deactivated" and then requires a fresh
      # arm sequence to redraw — display dead from the next override on. Hold
      # the override until longitudinal resumes (CC.longActive) or disengages
      # (cruiseState.enabled tracks longitudinal on this port).
      if CC.longActive and CS.out.gasPressed:
        self.acc_gas_override = True
      elif CC.longActive or not CS.out.cruiseState.enabled:
        self.acc_gas_override = False

    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        if self.CP.flags & VolkswagenFlags.MEB:
          accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX))
          accel, acc_status, acc_hold_type, braking_to_stop, leaving_standstill = self.meb_long_state.update(CS, CC, accel)
          can_sends.extend(mebcan.create_acc_accel_control(self.packer_pt, self.CAN.pt, self.CCP, CS.acc_type, CC.enabled,
                                                           accel, acc_status, acc_hold_type, braking_to_stop, leaving_standstill,
                                                           CS.out.vEgoRaw * CV.MS_TO_KPH, CS.travel_assist_available))

        else:
          stopping = actuators.longControlState == LongCtrlState.stopping
          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
          if self.CP.flags & VolkswagenFlags.MLB and not CS.out.accFaulted:
            # Stock J428 keeps ACC_Status_ACC=4 for the WHOLE gas override (B8PA rlog
            # route 0000000f t=872.3-878), matching ACC_02 Status_Anzeige=4. Sending
            # 2 (standby) here while ACC_02 says 4 is inconsistent, and the Kombi drops
            # the ACC display ~2s into the override. accel/limits stay gated by
            # CC.longActive below — only the status value reflects the override.
            acc_long_active = CC.longActive or self.acc_gas_override
            # ANB (stock AEB) active: exit ACC regulation entirely until the ANB
            # cooldown in carState.stockAeb expires. The ESP AWV consistency
            # monitor counts ACC-regulating + ANB-intervening overlap and
            # permanently faults TSK/ACC past a threshold, so standby (accel
            # zeroed below, main switch stays on) is the only safe claim.
            # Lateral is unaffected. panda additionally blocks any active
            # ACC_01 frames during this window as a backstop.
            acc_long_active = acc_long_active and not CS.out.stockAeb
            acc_control = 4 if (acc_long_active and CS.out.gasPressed) else \
                          self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, acc_long_active)
          mlb_anb_hold = self.CP.flags & VolkswagenFlags.MLB and CS.out.stockAeb
          accel_active = CC.longActive and not mlb_anb_hold
          accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if accel_active else 0)
          stopping = stopping and accel_active
          starting = accel_active and actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < 0.25)
          can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, accel_active, accel,
                                                             acc_control, stopping, starting, CS.esp_hold_confirmation))

        self.accel_last = accel

      #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      if self.CP.flags & VolkswagenFlags.MLB:
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control,
                                                         v_ego=CS.out.vEgo))
        # B8 Kombi lane-keep lamp follows camera msg 0x30A byte2, not LDW_02 LED bits;
        # byte3/byte1 carry the FIS lane-line graphic (solid lines / departure warning)
        can_sends.append(self.CCS.create_lka_lamp_control(self.packer_pt, self.CAN.pt, CC.latActive,
                                                          CS.out.steeringPressed, v_ego=CS.out.vEgo,
                                                          departing=hud_control.leftLaneDepart or hud_control.rightLaneDepart,
                                                          lat_enabled=hud_control.latEnabled))
      else:
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control))

    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      self.distance_bar_frame = self.frame

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & VolkswagenFlags.MEB:
        fcw_alert = hud_control.visualAlert == VisualAlert.fcw
        show_distance_bars = self.frame - self.distance_bar_frame < 400
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:
          lead_distance = 8
        can_sends.append(mebcan.create_acc_hud_control(self.packer_pt, self.CAN.pt, self.meb_long_state.acc_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                       hud_control.leadVisible, hud_control.leadDistanceBars, show_distance_bars,
                                                       lead_distance, fcw_alert))

      else:
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
          lead_distance = 512 if CS.upscale_lead_car_signal else 8
        if self.CP.flags & VolkswagenFlags.MLB:
          # Gas override keeps ACC active on the cluster (stock Status=4 = "background
          # override", the Kombi shows "ACC:override" and KEEPS the lead-car graphic).
          # controlsd drops CC.longActive while overrideLongitudinal is present; using it
          # directly makes the HUD fall to Status=2 (standby) for the whole override, so
          # the Kombi hides the graphic ~2s later and never redraws it (no arm sequence).
          # During ANB (stockAeb) the HUD follows ACC_01 to standby: showing "active"
          # while the drivetrain frame claims standby would desync the Kombi state.
          hud_long_active = (CC.longActive or self.acc_gas_override) and not CS.out.stockAeb
          acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted,
                                                         hud_long_active, gas_pressed=CS.out.gasPressed)
        else:
          acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
        # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH

        # MLB:Logic for hud text, bottom acc text display
        if self.CP.flags & VolkswagenFlags.MLB:
          # Pre-active display arming: stock J428 switches the DISPLAY fields to the
          # active presentation ~0.1s before ACC_Status flips to 3 (pre-frames sent
          # the moment the driver presses SET/RES, while status is still 2 — B8PA
          # rlog route 0000000f t=885.26). The Kombi appears to require this
          # arm->activate sequence to (re)draw the lead-car graphic after a cancel.
          if any(b.pressed and b.type in (ButtonType.setCruise, ButtonType.resumeCruise)
                 for b in CS.out.buttonEvents):
            self.gra_arm_frame = self.frame
            self.texte_timer = self.frame + int(2.0 / DT_CTRL)
            self.mlb_hud_text = 21
          # Stock J428 sends a ~2s announcement (Display_Prio=2 + Texte) on EVERY
          # ACC activation, not just on set-speed changes (route 0000000a resume
          # at t=369.05). Without this the Kombi never redraws the lead-car
          # graphic after a cancel/resume cycle.
          if CC.longActive and not self.last_long_active:
            self.texte_timer = self.frame + int(2.0 / DT_CTRL)
            self.mlb_hud_text = 21
          self.last_long_active = CC.longActive
          if set_speed != self.last_set_speed:
            self.texte_timer = self.frame + int(2.0 / DT_CTRL)
            self.mlb_hud_text = 21
            self.last_set_speed = set_speed
          elif hud_control.leadDistanceBars != self.last_lead_distance_bars:
            self.texte_timer = self.frame + int(2.0 / DT_CTRL)
            self.mlb_hud_text = {1: 2, 2: 3, 3: 4, 4: 5}.get(hud_control.leadDistanceBars, 0)
            self.last_lead_distance_bars = hud_control.leadDistanceBars
          elif self.frame > self.texte_timer:
            self.mlb_hud_text = 0

        if self.CP.flags & VolkswagenFlags.MLB:
          display_armed = not CC.longActive and (self.frame - self.gra_arm_frame) <= int(0.5 / DT_CTRL)
          can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                           hud_control.leadDistance, hud_control, self.mlb_hud_text,
                                                           announcing=self.frame <= self.texte_timer,
                                                           display_armed=display_armed,
                                                           stock_relevant_obj=CS.stock_acc_relevant_obj,
                                                           stock_abstandsindex=CS.stock_acc_abstandsindex,
                                                           v_ego_kph=CS.out.vEgo * CV.MS_TO_KPH))
        else:
          can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                           lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel_last

    self.lead_distance_bars_last = hud_control.leadDistanceBars
    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
