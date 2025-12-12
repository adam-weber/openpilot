from opendbc.car import CanBusBase, structs

HUDControl = structs.CarControl.HUDControl


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset

  @property
  def radar(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2


def calculate_lat_ctl2_checksum(mode: int, counter: int, dat: bytearray) -> int:
  curvature = (dat[2] << 3) | ((dat[3]) >> 5)
  curvature_rate = (dat[6] << 3) | ((dat[7]) >> 5)
  path_angle = ((dat[3] & 0x1F) << 6) | ((dat[4]) >> 2)
  path_offset = ((dat[4] & 0x3) << 8) | dat[5]

  checksum = mode + counter
  for sig_val in (curvature, curvature_rate, path_angle, path_offset):
    checksum += sig_val + (sig_val >> 8)

  return 0xFF - (checksum & 0xFF)


def create_lka_msg(packer, CAN: CanBus, lat_active: bool, hud_control):
  """
  Creates an empty CAN message for the Ford LKA Command.

  This command can apply "Lane Keeping Aid" maneuvers, which are subject to the PSCM lockout.

  Frequency is 33Hz.

  # Example hud_control data:
  hud_control: (
    speedVisible = false,
    setSpeed = 12.96416,
    lanesVisible = true,
    leadVisible = false,
    visualAlert = none,
    audibleAlert = none,
    rightLaneVisible = true,
    leftLaneVisible = true,
    rightLaneDepart = false,
    leftLaneDepart = false,
    leadDistanceBars = 3
  )

  """
  return packer.make_can_msg("Lane_Assist_Data1", CAN.main, {})



def create_lat_ctl_msg(packer, CAN: CanBus, lat_active: bool, ramp_type: int, precision_type: int, path_offset: float, path_angle: float,
                       curvature: float, curvature_rate: float):
  """
  Creates a CAN message for the Ford TJA/LCA Command.

  This command can apply "Lane Centering" maneuvers: continuous lane centering for traffic jam assist and highway
  driving. It is not subject to the PSCM lockout.

  Ford lane centering command uses a third order polynomial to describe the road centerline. The polynomial is defined
  by the following coefficients:
    c0: lateral offset between the vehicle and the centerline (positive is right)
    c1: heading angle between the vehicle and the centerline (positive is right)
    c2: curvature of the centerline (positive is left)
    c3: rate of change of curvature of the centerline
  As the PSCM combines this information with other sensor data, such as the vehicle's yaw rate and speed, the steering
  angle cannot be easily controlled.

  The PSCM should be configured to accept TJA/LCA commands before these commands will be processed. This can be done
  using tools such as Forscan.

  Frequency is 20Hz.
  """

  values = {
    "LatCtlRng_L_Max": 0,                       # Unknown [0|126] meter
    "HandsOffCnfm_B_Rq": 0,                     # Unknown: 0=Inactive, 1=Active [0|1]
    "LatCtl_D_Rq": 1 if lat_active else 0,      # Mode: 0=None, 1=ContinuousPathFollowing, 2=InterventionLeft,
                                                #       3=InterventionRight, 4-7=NotUsed [0|7]
    "LatCtlRampType_D_Rq": ramp_type,           # Ramp speed: 0=Slow, 1=Medium, 2=Fast, 3=Immediate [0|3]
                                                #             Makes no difference with curvature control
    "LatCtlPrecision_D_Rq": precision_type,     # Precision: 0=Comfortable, 1=Precise, 2/3=NotUsed [0|3]
                                                #            The stock system always uses comfortable
    "LatCtlPathOffst_L_Actl": path_offset,      # Path offset [-5.12|5.11] meter
    "LatCtlPath_An_Actl": path_angle,           # Path angle [-0.5|0.5235] radians
    "LatCtlCurv_NoRate_Actl": curvature_rate,   # Curvature rate [-0.001024|0.00102375] 1/meter^2
    "LatCtlCurv_No_Actl": curvature,            # Curvature [-0.02|0.02094] 1/meter
  }
  return packer.make_can_msg("LateralMotionControl", CAN.main, values)


def create_lat_ctl2_msg(packer, CAN: CanBus, mode: int, ramp_type: int, precision_type: int, path_offset: float, path_angle: float, curvature: float,
                        curvature_rate: float, counter: int):
  """
  Create a CAN message for the new Ford Lane Centering command.

  This message is used on the CAN FD platform and replaces the old LateralMotionControl message. It is similar but has
  additional signals for a counter and checksum.

  Frequency is 20Hz.
  """

  values = {
    "LatCtl_D2_Rq": mode,                       # Mode: 0=None, 1=PathFollowingLimitedMode, 2=PathFollowingExtendedMode,
                                                #       3=SafeRampOut, 4-7=NotUsed [0|7]
    "LatCtlRampType_D_Rq": ramp_type,           # 0=Slow, 1=Medium, 2=Fast, 3=Immediate [0|3]
    "LatCtlPrecision_D_Rq": precision_type,     # 0=Comfortable, 1=Precise, 2/3=NotUsed [0|3]
    "LatCtlPathOffst_L_Actl": path_offset,      # [-5.12|5.11] meter
    "LatCtlPath_An_Actl": path_angle,           # [-0.5|0.5235] radians
    "LatCtlCurv_No_Actl": curvature,            # [-0.02|0.02094] 1/meter
    "LatCtlCrv_NoRate2_Actl": curvature_rate,   # [-0.001024|0.001023] 1/meter^2
    "HandsOffCnfm_B_Rq": 0,                     # 0=Inactive, 1=Active [0|1]
    "LatCtlPath_No_Cnt": counter,               # [0|15]
    "LatCtlPath_No_Cs": 0,                      # [0|255]
  }

  # calculate checksum
  dat = packer.make_can_msg("LateralMotionControl2", 0, values)[1]
  values["LatCtlPath_No_Cs"] = calculate_lat_ctl2_checksum(mode, counter, dat)

  return packer.make_can_msg("LateralMotionControl2", CAN.main, values)


def create_angle_control_msg(packer, CAN: CanBus, angle_deg: float, enabled: bool, sapp_state: int, sapp_angle_req: int):
  """
  Creates a CAN message for Ford angle control via ParkAid_Data (SAPP).

  Args:
    angle_deg: Desired steering wheel angle in degrees (-1000 to +1000)
    enabled: Whether angle control is active
    sapp_state: SAPP handshake state (0=inactive, 1=initializing, 2=active)
    sapp_angle_req: Toggles 0/1 to indicate new angle request

  Frequency is 50Hz (message ID 0x3A8).
  """
  values = {
    "ExtSteeringAngleReq2": angle_deg,           # Steering angle in degrees
    "EPASExtAngleStatReq": 1 if enabled else 0,  # Enable angle control
    "ApaSys_D_Stat": sapp_state,                 # SAPP system state (handshake)
    # All other signals default to 0/inactive
    "SAPPStatusCoding": 0,
    "ApaSteWhl_D_RqDrv": 0,
    "ApaSteScanMde_D_Stat": 0,
    "ApaSelSapp_D_Stat": 1,  # CRITICAL: Tell PSCM that SAPP is "Selectable"!
    "ApaSelPpa_D_Stat": 0,
    "ApaSelPoa_D_Stat": 0,
    "ApaScan_D_Stat": 0,
    "ApaLongCtl_D_RqDrv": 0,
    "ApaGearShif_D_RqDrv": 0,
    "ApaActvSide2_D_Stat": 0,
    "ApaAcsy_D_RqDrv": 0,
    "ApaTrgtDist_D_Stat": 0,
    "ApaMsgTxt_D_Rq": 0,
    "ApaChime_D_Rq": 0,
    "ApaButtnPrssd_B_Stat": 0,
  }

  return packer.make_can_msg("ParkAid_Data", CAN.camera, values)


def create_pam_status_msg(packer, CAN: CanBus, sapp_active: bool):
  """
  Creates ParkAid_Aud_Warn_Stat message (0x3AA/938) - PAM heartbeat.

  This message tells PSCM that the Park Assist Module (PAM) is alive and
  what mode is available/active. Critical for SAPP to work continuously!

  Args:
    sapp_active: True if SAPP mode is currently active (Mode 1 engaged)

  Frequency: 50Hz (same as ParkAid_Data)
  """
  values = {
    # Tell PSCM what modes are available (4 = SAPP only)
    "ApaMde_D_Avail": 4,  # SAPP available
    # Tell PSCM current mode (2 = SAPP active, 1 = Off)
    "ApaMde_D_Stat": 2 if sapp_active else 1,
    # All other signals default to 0/inactive
    "PrkAidMsgTxt_D_Rq": 0,
    "ApaActvSd_D_Actl": 0,
    "PrkAidSwtch_B_Stat": 0,
    "RpaChime_D_Rq": 0,
    "FpaChime_D_Rq": 0,
    "PrkBrkEl_B_RqFap": 0,
    "PrkAidSnsFlCntr_D_Stat": 0,
    "PrkAidSnsFlCrnr_D_Stat": 0,
    "PrkAidSnsFrCntr_D_Stat": 0,
    "PrkAidSnsFrCrnr_D_Stat": 0,
    "SidePrkSnsL1_D_Stat": 0,
    "SidePrkSnsL2_D_Stat": 0,
    "SidePrkSnsR1_D_Stat": 0,
    "SidePrkSnsR2_D_Stat": 0,
    "PrkAidAudioMute_B_Rq": 0,
  }

  return packer.make_can_msg("ParkAid_Aud_Warn_Stat", CAN.camera, values)


def create_pam_status2_msg(packer, CAN: CanBus):
  """
  Creates ParkAid_Aud_Warn_Stat2 message (0x3AB/939) - Additional PAM status.

  This is the THIRD required PAM message. Without it, PSCM sets DTC 0xC159
  "Lost Communication With Parking Assist Control Module" after 5 seconds!

  Frequency: 50Hz (same as other PAM messages)
  """
  values = {
    # Parking sensor statuses - all inactive (no sensors detected)
    "PrkAidSnsRlCrnr_D_Stat": 0,
    "PrkAidSnsRrCntr_D_Stat": 0,
    "PrkAidSnsRrCrnr_D_Stat": 0,
    "PrkAidSnsRlCntr_D_Stat": 0,
    "SidePrkSnsL3_D_Stat": 0,
    "SidePrkSnsL4_D_Stat": 0,
    "SidePrkSnsR3_D_Stat": 0,
    "SidePrkSnsR4_D_Stat": 0,
    # System status
    "PrkAid_D_Falt": 0,  # No fault
    "PrkAidFront_D_Stat": 0,  # Front sensors inactive
    "PrkAidRear_D_Stat": 0,  # Rear sensors inactive
    "PrkAidChime_D_Stat": 0,  # No chime
    # Brake requests - all inactive
    "ApaLongCtrlEnbl_D_Rq": 0,  # No longitudinal control
    "ApaBrk_A_Rq": 0,  # No brake request
    "ApaBrk_D_Rq": 0,  # No brake request
  }

  return packer.make_can_msg("ParkAid_Aud_Warn_Stat2", CAN.camera, values)


def create_speed_spoof_msg(packer, CAN: CanBus, speed_kph: float, counter: int, gear_reverse: bool):
  """
  Creates spoofed speed message for EngVehicleSpThrottle2 (0x202).
  Sent on camera bus (bus 2) to spoof PSCM into allowing angle control at higher speeds.

  Args:
    speed_kph: Spoofed vehicle speed in km/h (usually 0.0 for full spoofing)
    counter: Message counter (0-15)
    gear_reverse: True if in reverse gear

  Frequency is 50Hz.
  """
  # Checksum calculation (simple sum)
  cs = int((counter + (speed_kph * 100)) % 256)

  values = {
    "VehVTrlrAid_B_Avail": 1 if gear_reverse else 0,
    "VehVActlEng_No_Cs": cs,
    "VehVActlEng_No_Cnt": counter,
    "Veh_V_RqCcSet": 0,
    "VehVActlEng_D_Qf": 3,  # Quality factor: valid
    "Veh_V_ActlEng": speed_kph,
    "GearRvrse_D_Actl": 3 if gear_reverse else 1,
    "StrtrMtrCtlMsgTxt_D2_Rq": 0,
    "StrtrMtrCtlMsgTxt_D_Rq": 0,
    "StrtrMtrDlyStrt_B_Stat": 0,
  }

  return packer.make_can_msg("EngVehicleSpThrottle2", CAN.camera, values)


def create_brake_speed_spoof_msg(packer, CAN: CanBus, speed_kph: float, counter: int):
  """
  Creates spoofed speed message for BrakeSysFeatures (0x415).
  Sent on camera bus (bus 2) to spoof PSCM into allowing angle control at higher speeds.

  Args:
    speed_kph: Spoofed vehicle speed in km/h (usually 0.0 for full spoofing)
    counter: Message counter (0-15)

  Frequency is 50Hz.
  """
  # Checksum calculation (simple sum)
  cs = int((counter + (speed_kph * 100)) % 256)

  values = {
    "Veh_V_ActlBrk": speed_kph,
    "LsmcBrkDecel_D_Stat": 0,
    "VehVActlBrk_No_Cs": cs,
    "VehVActlBrk_No_Cnt": counter,
    "VehVActlBrk_D_Qf": 3,  # Quality factor: valid
    "BrkFluidLvl_D_Stat": 0,
    "VehYawLin_W_Rq": 0,
    "VehYawNonLin_W_Rq": 0,
    "VehStab_D_Stat": 0,
  }

  return packer.make_can_msg("BrakeSysFeatures", CAN.camera, values)


def create_acc_msg(packer, CAN: CanBus, long_active: bool, gas: float, accel: float, stopping: bool, brake_request, v_ego_kph: float):
  """
  Creates a CAN message for the Ford ACC Command.

  This command can be used to enable ACC, to set the ACC gas/brake/decel values
  and to disable ACC.

  Frequency is 50Hz.
  """
  values = {
    "AccBrkTot_A_Rq": accel,                          # Brake total accel request: [-20|11.9449] m/s^2
    "Cmbb_B_Enbl": 1 if long_active else 0,           # Enabled: 0=No, 1=Yes
    "AccPrpl_A_Rq": gas,                              # Acceleration request: [-5|5.23] m/s^2
    # No observed acceleration seen from this signal alone. During stock system operation, it appears to
    # be the raw acceleration request (AccPrpl_A_Rq when positive, AccBrkTot_A_Rq when negative)
    "AccPrpl_A_Pred": -5.0,                           # Acceleration request: [-5|5.23] m/s^2
    "AccResumEnbl_B_Rq": 1 if long_active else 0,
    # No observed acceleration seen from this signal alone
    "AccVeh_V_Trg": v_ego_kph,                        # Target speed: [0|255] km/h
    # TODO: we may be able to improve braking response by utilizing pre-charging better
    # When setting these two bits without AccBrkTot_A_Rq, an initial jerk is observed and car may be able to brake temporarily with AccPrpl_A_Rq
    "AccBrkPrchg_B_Rq": 1 if brake_request else 0,            # Pre-charge brake request: 0=No, 1=Yes
    "AccBrkDecel_B_Rq": 1 if brake_request else 0,            # Deceleration request: 0=Inactive, 1=Active
    "AccStopStat_B_Rq": 1 if stopping else 0,
  }
  return packer.make_can_msg("ACCDATA", CAN.main, values)


def create_acc_ui_msg(packer, CAN: CanBus, CP, main_on: bool, enabled: bool, fcw_alert: bool, standstill: bool,
                      hud_control, stock_values: dict, send_hands_free_msg: bool, send_ui: bool, send_bars: bool, tja_warn: int, tja_msg: int, test_mode_beep: bool = False):
  """
  Creates a CAN message for the Ford IPC adaptive cruise, forward collision warning and traffic jam
  assist status.

  Stock functionality is maintained by passing through unmodified signals.

  Frequency is 5Hz.
  """

  # Tja_D_Stat
  if enabled:
    if hud_control.leftLaneDepart:
      status = 3  # ActiveInterventionLeft
    elif hud_control.rightLaneDepart:
      status = 4  # ActiveInterventionRight
    elif send_hands_free_msg:
      status = 7 # Show BlueCruise UI in the Cluster
    else:
      status = 2  # Active
  elif main_on:
    if hud_control.leftLaneDepart:
      status = 5  # ActiveWarningLeft
    elif hud_control.rightLaneDepart:
      status = 6  # ActiveWarningRight
    else:
      status = 1  # Standby
  elif standstill:
    status = 0  # Off
  else:
    status = 1    # Standby

  values = {s: stock_values[s] for s in [
    "HaDsply_No_Cs",
    "HaDsply_No_Cnt",
    "AccStopStat_D_Dsply",       # ACC stopped status message
    "AccTrgDist2_D_Dsply",       # ACC target distance
    "AccStopRes_B_Dsply",
    #"TjaWarn_D_Rq",              # TJA warning
    #"TjaMsgTxt_D_Dsply",         # TJA text
    "IaccLamp_D_Rq",             # iACC status icon
    "AccMsgTxt_D2_Rq",           # ACC text
    "FcwDeny_B_Dsply",           # FCW disabled
    "FcwMemStat_B_Actl",         # FCW enabled setting
    "AccTGap_B_Dsply",           # ACC time gap display setting
    "CadsAlignIncplt_B_Actl",
    "AccFllwMde_B_Dsply",        # ACC follow mode display setting
    "CadsRadrBlck_B_Actl",
    "CmbbPostEvnt_B_Dsply",      # AEB event status
    "AccStopMde_B_Dsply",        # ACC stop mode display setting
    "FcwMemSens_D_Actl",         # FCW sensitivity setting
    "FcwMsgTxt_D_Rq",            # FCW text
    "AccWarn_D_Dsply",           # ACC warning
    "FcwVisblWarn_B_Rq",         # FCW visible alert
    "FcwAudioWarn_B_Rq",         # FCW audio alert
    "AccTGap_D_Dsply",           # ACC time gap
    "AccMemEnbl_B_RqDrv",        # ACC adaptive/normal setting
    "FdaMem_B_Stat",             # FDA enabled setting
  ]}

  values.update({
    "Tja_D_Stat": status,        # TJA status
    "TjaWarn_D_Rq": tja_warn,    # TJA warning
    "TjaMsgTxt_D_Dsply": tja_msg,# TJA text
  })

  if CP.openpilotLongitudinalControl:
    values.update({
      "AccStopStat_D_Dsply": 2 if standstill else 0,              # Stopping status text
      "AccMsgTxt_D2_Rq": 0,                                       # ACC text
      "AccTGap_B_Dsply": 1 if send_bars else 0,      	            # Show time gap control UI
      "AccFllwMde_B_Dsply": 1 if hud_control.leadVisible else 0,  # Lead indicator
      "AccStopMde_B_Dsply": 1 if standstill else 0,
      "AccWarn_D_Dsply": 0,                                       # ACC warning
      "AccTGap_D_Dsply": hud_control.leadDistanceBars,            # Time gap
    })

  # Forwards FCW alert from IPMA
  if fcw_alert:
    values["FcwVisblWarn_B_Rq"] = 1  # FCW visible alert
    values["FcwAudioWarn_B_Rq"] = 1  # FCW audio alert

  # Test mode beep (Mode 1 activation)
  if test_mode_beep:
    values["FcwAudioWarn_B_Rq"] = 1  # Audio beep

  return packer.make_can_msg("ACCDATA_3", CAN.main, values)


def create_lkas_ui_msg(packer, CAN: CanBus, main_on: bool, enabled: bool, hands: int, hud_control,
                       stock_values: dict):
  """
  Creates a CAN message for the Ford IPC IPMA/LKAS status.

  Show the LKAS status with the "driver assist" lines in the IPC.

  Stock functionality is maintained by passing through unmodified signals.

  Frequency is 1Hz.

  # Example hud_control data:
  hud_control: (
    speedVisible = false,
    setSpeed = 12.96416,
    lanesVisible = true,
    leadVisible = false,
    visualAlert = none,
    audibleAlert = none,
    rightLaneVisible = true,
    leftLaneVisible = true,
    rightLaneDepart = false,
    leftLaneDepart = false,
    leadDistanceBars = 3
  )

  # LaActvStats_D_Dsply Value Table
  # Left \\ Right  | Intervene | Warning | Suppress | Available | None
  # ---------------------------------------------------------------
  # Intervene     | 24        | 19      | 14       | 9         | 4
  # Warning       | 23        | 18      | 13       | 8         | 3
  # Suppress      | 22        | 17      | 12       | 7         | 2
  # Available     | 21        | 16      | 11       | 6         | 1
  # None          | 20        | 15      | 10       | 5         | 0

  """

  # TODO: test suppress state
  lines = 0

  if hud_control is not None:
    left_status = 2  # Default to Suppress
    right_status = 10  # Default to Suppress

    # Determine left lane status
    if hud_control.leftLaneDepart:
      left_status = 4 # Intervene (Yellow)
      #left_status = 3 # Warning (Red)
    elif hud_control.leftLaneVisible:
      left_status = 1 # Available
    else:
      left_status = 2 # Suppress

    # Determine right lane status
    if hud_control.rightLaneDepart:
      right_status = 20 # Intervene (Yellow)
      #right_status = 15 # Warning (Red)
    elif hud_control.rightLaneVisible:
      right_status = 5
    else:
      right_status = 10 # Suppress

    # Combine left and right lane status
    lines = left_status + right_status
  else:
    #Set this to 0 if hud_control is None
    lines = 0

  values = {s: stock_values[s] for s in [
    "FeatConfigIpmaActl",
    "FeatNoIpmaActl",
    "PersIndexIpma_D_Actl",
    "AhbcRampingV_D_Rq",     # AHB ramping
    "LaDenyStats_B_Dsply",   # LKAS error
    "CamraDefog_B_Req",      # Windshield heater?
    "CamraStats_D_Dsply",    # Camera status
    "DasAlrtLvl_D_Dsply",    # DAS alert level
    "DasStats_D_Dsply",      # DAS status
    "DasWarn_D_Dsply",       # DAS warning
    "AhbHiBeam_D_Rq",        # AHB status
    "Passthru_63",
    "Passthru_48",
  ]}

  values.update({
    "LaActvStats_D_Dsply": lines,                 # LKAS status (lines) [0|31]
    "LaHandsOff_D_Dsply": hands,   # 0=HandsOn, 1=Level1 (w/o chime), 2=Level2 (w/ chime), 3=Suppressed
  })
  return packer.make_can_msg("IPMA_Data", CAN.main, values)


def create_button_msg(packer, bus: int, stock_values: dict, cancel=False, resume=False, tja_toggle=False, icbm_button=None):
  """
  Creates a CAN message for the Ford SCCM buttons/switches.

  Includes cruise control buttons, turn lights and more.

  Frequency is 10Hz.

  Args:
    icbm_button: Optional string signal name for ICBM button press (e.g., "CcAslButtnSetIncPress", "CcAslButtnSetDecPress")
  """

  values = {s: stock_values[s] for s in [
    "HeadLghtHiFlash_D_Stat",  # SCCM Passthrough the remaining buttons
    "TurnLghtSwtch_D_Stat",    # SCCM Turn signal switch
    "WiprFront_D_Stat",
    "LghtAmb_D_Sns",
    "AccButtnGapDecPress",
    "AccButtnGapIncPress",
    "AslButtnOnOffCnclPress",
    "AslButtnOnOffPress",
    "LaSwtchPos_D_Stat",
    "CcAslButtnCnclResPress",
    "CcAslButtnDeny_B_Actl",
    "CcAslButtnIndxDecPress",
    "CcAslButtnIndxIncPress",
    "CcAslButtnOffCnclPress",
    "CcAslButtnOnOffCncl",
    "CcAslButtnOnPress",
    "CcAslButtnResDecPress",
    "CcAslButtnResIncPress",
    "CcAslButtnSetDecPress",
    "CcAslButtnSetIncPress",
    "CcAslButtnSetPress",
    "CcButtnOffPress",
    "CcButtnOnOffCnclPress",
    "CcButtnOnOffPress",
    "CcButtnOnPress",
    "HeadLghtHiFlash_D_Actl",
    "HeadLghtHiOn_B_StatAhb",
    "AhbStat_B_Dsply",
    "AccButtnGapTogglePress",
    "WiprFrontSwtch_D_Stat",
    "HeadLghtHiCtrl_D_RqAhb",
  ]}

  values.update({
    "CcAslButtnCnclPress": 1 if cancel else 0,      # CC cancel button
    "CcAsllButtnResPress": 1 if resume else 0,      # CC resume button
    "TjaButtnOnOffPress": 1 if tja_toggle else 0,   # LCA/TJA toggle button
  })

  # ICBM button support - set the specified button signal to 1
  if icbm_button is not None:
    values[icbm_button] = 1

  return packer.make_can_msg("Steering_Data_FD1", bus, values)
