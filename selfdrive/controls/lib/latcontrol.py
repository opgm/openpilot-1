from abc import abstractmethod, ABC

from common.numpy_fast import clip
from common.realtime import DT_CTRL
from typing import Type

from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.controls.lib.vehicle_model import VehicleModel

MIN_STEER_SPEED = 0.3


class LatControl(ABC):
  def __init__(self, CP, CI: Type[CarInterfaceBase]):
    self.sat_count_rate = 1.0 * DT_CTRL
    self.sat_limit: float = CP.steerLimitTimer
    self.sat_count = 0.

    # we define the steer torque scale as [-1.0...1.0]
    self.steer_max = 1.0

  @abstractmethod
  def update(self, active: bool, CS, VM: VehicleModel, params,
             last_actuators, desired_curvature: float, desired_curvature_rate: float, llk):
    pass

  def reset(self):
    self.sat_count = 0.

  def _check_saturation(self, saturated: bool, CS) -> bool:
    if saturated and CS.vEgo > 10. and not CS.steeringRateLimited and not CS.steeringPressed:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate
    self.sat_count = clip(self.sat_count, 0.0, self.sat_limit)
    return self.sat_count > (self.sat_limit - 1e-3)
