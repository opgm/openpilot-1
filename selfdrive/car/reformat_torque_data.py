from collections import defaultdict
import json
import os
from typing import Dict, DefaultDict, Tuple, List

import nacl
from common.basedir import BASEDIR
from selfdrive.car.car_helpers import interface_names
# Convert torque_data.json to per-brand file
# Flip the organization from attrib: model to model: attrib
# Sort and make human readable
# (Maybe) replace the non-standards compliant bare NaN with quoted
# TODO: update reader to remove quotes...
# TODO: write files to folders by brand
# TODO: update helpers to also load torque with interface

class TorqueFormatter:
  TORQUE_PARAMS_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data.json')
  BRAND_PATH: Dict[str, str] = {brand: f'selfdrive/car/{brand}/torque_data.json' for brand in interface_names}
  MODEL_BRAND: Dict[str, str] = {model: brand for brand in interface_names for model in interface_names[brand]}
  
  def __init__(self):
    with open(TorqueFormatter.TORQUE_PARAMS_PATH) as f:
      dataStr = f.read()
    dataStr = dataStr.replace("\"NaN\"","NaN") # Just in case the nans are escaped
    self.source_data: Dict[str,Dict[str,float]] = json.loads(dataStr)
    
    self.new_data: DefaultDict[str,DefaultDict[str,DefaultDict[str,float]]] = \
      defaultdict(
        lambda: defaultdict(
          lambda: defaultdict(
            lambda: float('NaN'))
          )
        )
      
  def _lookup_brand(self, model: str) -> str:
    if model in TorqueFormatter.MODEL_BRAND:
      return TorqueFormatter.MODEL_BRAND[model]
    
    for brand in TorqueFormatter.BRAND_PATH:
      if model.lower().startswith(brand.lower()):
        return brand
    
    raise f"Ubable to determine brand for {model}"

  def process(self):
    for param in self.source_data:
      for model in self.source_data[param]:
        value = self.source_data[param].get(model)
        brand = self._lookup_brand(model)
        self.new_data[brand][model][param] = value

  def write_combined(self, path: str, escape_nan = False):
    with open(path,'w') as f:
      out = json.dumps(self.new_data, sort_keys=True, indent=2)
      if escape_nan:
        out = out.replace("NaN","\"NaN\"")
      f.write(out)


if __name__ == "__main__":
  util = TorqueFormatter()
  util.process()
  util.write_combined(TorqueFormatter.TORQUE_PARAMS_PATH + ".new.json")