import math

from cereal import log
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController

class LatControlPID(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_d=(CP.lateralTuning.pid.kdBP, CP.lateralTuning.pid.kdV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    
    self.last_curve_is_right = False
    self.lateralTuneSplit = CP.lateralTuneSplit # storing to detect change

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, VM, params, last_actuators, desired_curvature, desired_curvature_rate, llk):
        
    # TODO: JJS: Ensure that changes to CP are reflected in PI controller
    # TODO: JJS: Find a way for pid to read from something mutable directly, rather than comparing every time

    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)
    pid_log.usingRightTune = False

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
    error = angle_steers_des - CS.steeringAngleDeg

    # This is messy. Normally not divided and we are using pid
    # If lateralTuneSplit changes to false when it was True
    # We need to switch from pidRight to pid
    # I'm sure there is a better way
    # TODO: JJS: If this works, need to abstract at the latcontrol level as well as controlsd

    LR_SPLIT_PT = 0.0002 # Radians? to offset the zero pt for L/R split.  PID should use the left tune
    # TODO: Parameterize the split point?

    if (self.lateralTuneSplit != self.CP.lateralTuneSplit):
      if not self.CP.lateralTuneSplit: # Split tune was disabled live - update to left tune
        self.pid.update_params(k_f=self.CP.lateralTuning.pid.kf,
                              k_p=(self.CP.lateralTuning.pid.kpBP, self.CP.lateralTuning.pid.kpV),
                              k_i=(self.CP.lateralTuning.pid.kiBP, self.CP.lateralTuning.pid.kiV),
                              k_d=(self.CP.lateralTuning.pid.kdBP, self.CP.lateralTuning.pid.kdV)
                              )
      self.lateralTuneSplit = self.CP.lateralTuneSplit

    

    if self.CP.lateralTuneSplit:
      # Cannot use error; error will be negative any time the wheel moves to the left
      # We are only concerned with instances where the car needs the wheel right of center
      # To prevent nasty flip-flopping between tunes, LR_SPLIT_PT should apply a non-zero split point.
      # The sign of the split point biases the opposite sign around zero
      # TODO: JJS: look at raw values for LateralPIDState.steeringAngleDesiredDeg
      # When the wheel is close to center there will probably be an eyeball range
      # +0.002 on the highway is enough to saturate
      # Right is positive, left is negative
      # Note that torque is backwards
      curve_is_right = desired_curvature >= LR_SPLIT_PT
      pid_log.usingRightTune = curve_is_right

      # Check for changes in kf
      if curve_is_right:
        # k_f is immutable, and PI is too abstract for using a CP reference
        # Because the pid controller is in-process with controlsd,
        # and the livetuner edits the tuning lists in a mutable fashion
        # we don't need to update anything but kf
        if not math.isclose(self.pid.k_f, self.CP.lateralTuningRight.pid.kf):
          self.pid.k_f = self.CP.lateralTuningRight.pid.kf
      else:
        if not math.isclose(self.pid.k_f, self.CP.lateralTuning.pid.kf):
          self.pid.k_f = self.CP.lateralTuning.pid.kf


      
      if self.last_curve_is_right != curve_is_right: # We changed direction!
        if curve_is_right:
          self.pid.update_params(k_f=self.CP.lateralTuningRight.pid.kf,
                                k_p=(self.CP.lateralTuningRight.pid.kpBP, self.CP.lateralTuningRight.pid.kpV),
                                k_i=(self.CP.lateralTuningRight.pid.kiBP, self.CP.lateralTuningRight.pid.kiV),
                                k_d=(self.CP.lateralTuningRight.pid.kdBP, self.CP.lateralTuningRight.pid.kdV)
                                )
        else:
          self.pid.update_params(k_f=self.CP.lateralTuning.pid.kf,
                                k_p=(self.CP.lateralTuning.pid.kpBP, self.CP.lateralTuning.pid.kpV),
                                k_i=(self.CP.lateralTuning.pid.kiBP, self.CP.lateralTuning.pid.kiV),
                                k_d=(self.CP.lateralTuning.pid.kdBP, self.CP.lateralTuning.pid.kdV)
                                )

        self.last_curve_is_right = curve_is_right


    pid_log.steeringAngleDesiredDeg = angle_steers_des
    pid_log.angleError = error
    
    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      # TODO: JJS: feedforward function may need to be splitable as well
      # offset does not contribute to resistive torque
      steer_feedforward = self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)

      output_steer = self.pid.update(error, override=CS.steeringPressed,
                                     feedforward=steer_feedforward, speed=CS.vEgo)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_steer) < 1e-3, CS)

    return output_steer, angle_steers_des, pid_log
