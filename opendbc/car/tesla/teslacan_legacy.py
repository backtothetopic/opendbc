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

  # tesla-unity IC integration: show AP-style blue lane lines on instrument cluster.
  # V1 minimum: straight lanes, no curve fit. Sends DAS_lanes (0x239) at 10Hz and
  # DAS_status (0x399) at 2Hz. Engagement-state-gated so lines appear/disappear with openpilot.

  def create_lane_message(self, enabled, counter):
    # uses DAS_lanes definition already in tesla_can.dbc
    values = {
      "DAS_leftLaneExists": 1 if enabled else 0,
      "DAS_rightLaneExists": 1 if enabled else 0,
      "DAS_virtualLaneWidth": 3.7,          # meters
      "DAS_virtualLaneViewRange": 50,       # meters
      "DAS_virtualLaneC0": 0.0,             # lateral offset (m)
      "DAS_virtualLaneC1": 0.0,             # heading (rad)
      "DAS_virtualLaneC2": 0.0,             # curvature (1/m)
      "DAS_virtualLaneC3": 0.0,             # curvature rate (1/m^2)
      "DAS_leftLineUsage": 2 if enabled else 0,   # 0=none, 2=valid
      "DAS_rightLineUsage": 2 if enabled else 0,
      "DAS_leftFork": 0,
      "DAS_rightFork": 0,
      "DAS_lanesCounter": counter,
    }
    return self.packers[CANBUS.party].make_can_msg("DAS_lanes", CANBUS.party, values)

  def create_das_status(self, enabled, counter):
    # DAS_status (0x399) is not defined in xnor's tesla_can.dbc, so we pack raw bytes
    # per the tesla-unity DBC definition. Minimum viable payload: set DAS_autopilotState = 5
    # (active_nav) when engaged, 2 (available) otherwise. All other signals zeroed.
    # Signal layout (little-endian bits):
    #   bit 0-3   DAS_autopilotState        (4 bits)
    #   bit 52-55 DAS_statusCounter         (4 bits)
    #   bit 56-63 DAS_statusChecksum        (1 byte) -- left 0; HW1 IC does not validate
    autopilot_state = 5 if enabled else 2
    byte0 = autopilot_state & 0x0F
    byte6 = (counter & 0x0F) << 4
    data = bytes([byte0, 0, 0, 0, 0, 0, byte6, 0])
    return 0x399, data, CANBUS.party
