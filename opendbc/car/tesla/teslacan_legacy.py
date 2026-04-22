from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams


class TeslaCANRaven:
  def __init__(self, packers):
    self.packers = packers
    self.CCP = CarControllerParams
    self.jerk_upper = self.CCP.JERK_LIMIT_MAX
    self.jerk_lower = self.CCP.JERK_LIMIT_MIN

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, counter, angle, enabled):
    values = {
      "DAS_steeringControlCounter": counter,
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
    }

    data = self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active, gas_pressed):
    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    if active:
      set_speed = 0 if accel < 0 else V_CRUISE_MAX

    if gas_pressed:
      self.jerk_upper = self.jerk_lower = 0.0
    else:
      self.jerk_lower = max(self.jerk_lower - self.CCP.JERK_RAMP_RATE, self.CCP.JERK_LIMIT_MIN)
      self.jerk_upper = min(self.jerk_upper + self.CCP.JERK_RAMP_RATE, self.CCP.JERK_LIMIT_MAX)

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": self.jerk_lower,
      "DAS_jerkMax": self.jerk_upper,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }

    data = self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacMonitorCounter": counter,
      "APS_eacAllow": 1,
    }

    data = self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)

  def create_lane_message(self, counter, enabled):
    # DAS_lanes (0x239) — tells the instrument cluster to draw lane lines.
    # No checksum on this message in xnor's DBC. Phase 1: straight virtual lane,
    # constant width/range. Real polynomial curves can be wired in later.
    values = {
      "DAS_leftLaneExists": 1 if enabled else 0,
      "DAS_rightLaneExists": 1 if enabled else 0,
      "DAS_virtualLaneWidth": 3.7,        # ~US highway lane width
      "DAS_virtualLaneViewRange": 50,     # meters ahead
      "DAS_virtualLaneC0": 0.0,
      "DAS_virtualLaneC1": 0.0,
      "DAS_virtualLaneC2": 0.0,
      "DAS_virtualLaneC3": 0.0,
      "DAS_leftLineUsage": 2 if enabled else 0,   # 2 = HIGH_CONFIDENCE
      "DAS_rightLineUsage": 2 if enabled else 0,
      "DAS_leftFork": 0,
      "DAS_rightFork": 0,
      "DAS_lanesCounter": counter,
    }
    return self.packers[CANBUS.party].make_can_msg("DAS_lanes", CANBUS.party, values)

  def create_das_status(self, counter, enabled):
    # AutopilotStatus (0x399) — xnor's renamed DAS_status. Telling the IC that
    # autopilot is active is what unlocks blue lane rendering from DAS_lanes.
    # Checksum intentionally left at 0 (tesla-unity's proven pattern — the IC
    # does not validate this checksum on AP1).
    values = {
      "autopilotStatus": 3 if enabled else 2,   # 3=ACTIVE_1, 2=AVAILABLE
      "DAS_blindSpotRearLeft": 0,
      "DAS_blindSpotRearRight": 0,
      "DAS_fusedSpeedLimit": 0,                 # no speed-limit display yet
      "DAS_suppressSpeedWarning": 0,
      "DAS_summonObstacle": 0,
      "DAS_summonClearedGate": 0,
      "DAS_visionOnlySpeedLimit": 0,
      "DAS_heaterState": 0,
      "DAS_forwardCollisionWarning": 0,
      "DAS_autoparkReady": 0,
      "DAS_autoParked": 0,
      "DAS_autoparkWaitingForBrake": 0,
      "DAS_summonFwdLeashReached": 0,
      "DAS_summonRvsLeashReached": 0,
      "DAS_sideCollisionAvoid": 0,
      "DAS_sideCollisionWarning": 0,
      "DAS_sideCollisionInhibit": 0,
      "DAS_csaState": 2 if enabled else 1,      # 2=ACTIVE, 1=HEALTHY_IDLE
      "DAS_laneDepartureWarning": 0,
      "DAS_fleetSpeedState": 0,
      "DAS_autopilotHandsOnState": 2,           # 2=normal (no warning)
      "DAS_autoLaneChangeState": 0,
      "DAS_summonAvailable": 0,
      "DAS_statusCounter": counter,
      "DAS_statusChecksum": 0,
    }
    return self.packers[CANBUS.party].make_can_msg("AutopilotStatus", CANBUS.party, values)

  def create_das_status2(self, counter, enabled):
    # DAS_status2 (0x389) — companion to AutopilotStatus. ACC/radar/robustness
    # status fields. Checksum left at 0 per tesla-unity pattern.
    values = {
      "DAS_accSpeedLimit": 0,
      "DAS_pmmObstacleSeverity": 0,
      "DAS_pmmLoggingRequest": 0,
      "DAS_activationFailureStatus": 0,
      "DAS_pmmUltrasonicsFaultReason": 0,
      "DAS_pmmRadarFaultReason": 0,
      "DAS_pmmSysFaultReason": 0,
      "DAS_pmmCameraFaultReason": 0,
      "DAS_ACC_report": 1,
      "DAS_lssState": 0,
      "DAS_radarTelemetry": 1,
      "DAS_robState": 2,
      "DAS_driverInteractionLevel": 0,
      "DAS_ppOffsetDesiredRamp": 0.0,
      "DAS_longCollisionWarning": 0,
      "DAS_status2Counter": counter,
      "DAS_status2Checksum": 0,
    }
    return self.packers[CANBUS.party].make_can_msg("DAS_status2", CANBUS.party, values)
