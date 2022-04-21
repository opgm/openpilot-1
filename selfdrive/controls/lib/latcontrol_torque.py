import math

from cereal import log
from common.numpy_fast import interp
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.


LOW_SPEED_FACTOR = 200
JERK_THRESHOLD = 0.2


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.CP = CP
    self.pid = PIDController(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.ki, k_d=CP.lateralTuning.torque.kd,
                             k_f=CP.lateralTuning.torque.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    self.use_steering_angle = CP.lateralTuning.torque.useSteeringAngle
    self.friction = CP.lateralTuning.torque.friction
    
    
    #self.last_curve_is_right = False
    self.lateralTuneSplit = CP.lateralTuneSplit # storing to detect change

  def reset(self):
    super().reset()
    self.pid.reset()

  def detectChange(self):
    # math.isclose compares to 9 digits
    if not math.isclose(self.pid.k_f, self.CP.lateralTuning.torque.kf):
      self.pid.k_f = self.CP.lateralTuning.torque.kf

    if not math.isclose(self.pid._k_p[1][0], self.CP.lateralTuning.torque.kp):
      self.pid._k_p[1][0] = self.CP.lateralTuning.torque.kp
    
    if not math.isclose(self.pid._k_i[1][0], self.CP.lateralTuning.torque.ki):
      self.pid._k_i[1][0] = self.CP.lateralTuning.torque.ki
    
    if (self.use_steering_angle != self.CP.lateralTuning.torque.useSteeringAngle):
      self.use_steering_angle = self.CP.lateralTuning.torque.useSteeringAngle
    
    if not math.isclose(self.friction, self.CP.lateralTuning.torque.friction):
      self.friction = self.CP.lateralTuning.torque.friction
  

  def detectChangeRight(self):
    # math.isclose compares to 9 digits
    if not math.isclose(self.pid.k_f, self.CP.lateralTuningRight.torque.kf):
      self.pid.k_f = self.CP.lateralTuningRight.torque.kf

    if not math.isclose(self.pid._k_p[1][0], self.CP.lateralTuningRight.torque.kp):
      self.pid._k_p[1][0] = self.CP.lateralTuningRight.torque.kp
    
    if not math.isclose(self.pid._k_i[1][0], self.CP.lateralTuningRight.torque.ki):
      self.pid._k_i[1][0] = self.CP.lateralTuningRight.torque.ki
    
    if (self.use_steering_angle != self.CP.lateralTuningRight.torque.useSteeringAngle):
      self.use_steering_angle = self.CP.lateralTuningRight.torque.useSteeringAngle
    
    if not math.isclose(self.friction, self.CP.lateralTuningRight.torque.friction):
      self.friction = self.CP.lateralTuningRight.torque.friction



  def update(self, active, CS, VM, params, last_actuators, desired_curvature, desired_curvature_rate, llk):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.usingRightTune = False
    #self.detectChange()

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      pid_log.active = False
      if not active:
        self.pid.reset()
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      else:
        actual_curvature = llk.angularVelocityCalibrated.value[2] / CS.vEgo
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2

      LR_SPLIT_PT = 0.0002

      if (self.lateralTuneSplit != self.CP.lateralTuneSplit):
        if not self.CP.lateralTuneSplit: # Split tune was disabled live - update to left tune
          self.detectChange()
        self.lateralTuneSplit = self.CP.lateralTuneSplit
      
      if self.CP.lateralTuneSplit:
        curve_is_right = desired_curvature >= LR_SPLIT_PT
        pid_log.usingRightTune = curve_is_right

        if curve_is_right:
          self.detectChangeRight()
        else:
          self.detectChange()

        # #Note: Don't need to detect the change because we must do a full compare every time
        # if self.last_curve_is_right != curve_is_right: # We changed direction!
        #   if curve_is_right:
        #     self.detectChangeRight()
        #   else:
        #     self.detectChange()
        #   self.last_curve_is_right = curve_is_right



      setpoint = desired_lateral_accel + LOW_SPEED_FACTOR * desired_curvature
      measurement = actual_lateral_accel + LOW_SPEED_FACTOR * actual_curvature
      error = setpoint - measurement
      pid_log.error = error

      ff = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
      # convert friction into lateral accel units for feedforward
      friction_compensation = interp(desired_lateral_jerk, [-JERK_THRESHOLD, JERK_THRESHOLD], [-self.friction, self.friction])
      ff += friction_compensation / self.CP.lateralTuning.torque.kf
      output_torque = self.pid.update(error,
                                      override=CS.steeringPressed, feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=CS.steeringRateLimited)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS)
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
