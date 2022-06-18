from collections import defaultdict
import json
import os
from typing import Dict, DefaultDict
from common.basedir import BASEDIR
from selfdrive.car.car_helpers import interface_names
# Convert torque_data.json to per-brand file
# Flip the organization from attrib: model to model: attrib
# Sort and make human readable
# (Maybe) replace the non-standards compliant bare NaN with quoted
# TODO: update reader to remove quotes...
# TODO: write files to folders by brand
# TODO: update helpers to also load torque with interface



TORQUE_PARAMS_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data.json')

brand_path: Dict[str, str] = dict()
model_brand: Dict[str, str] = dict()
source_data = dict()
default_val: float = float('NaN')
new_data: DefaultDict[str,DefaultDict[str,DefaultDict[str,float]]] = defaultdict(lambda:defaultdict(lambda: defaultdict(lambda: default_val)))

def load_interfaces():
  for brand_name in interface_names:
    path = f'selfdrive.car.{brand_name}.torque_data.json'
    brand_path[brand_name] = path
    for model_name in interface_names[brand_name]:
      model_brand[model_name] = brand_name

def lookup_brand(model: str):
  if model in model_brand:
    return model_brand[model]
  
  for brand in interface_names:
    if model.lower().startswith(brand.lower()):
      return brand
  
  raise "Nope"
    

def fix_it():
  load_interfaces()
  with open(TORQUE_PARAMS_PATH) as f:
    source_data = json.load(f)
  
  for param in source_data:
    for model in source_data[param]:
      value = source_data[param].get(model)
      brand = lookup_brand(model)
      new_data[brand][model][param] = value

  with open(TORQUE_PARAMS_PATH + ".new",'w') as f:
    out = json.dumps(new_data, sort_keys=True, indent=2)
    #out = out.replace("NaN","\"NaN\"")
    f.write(out)
  
  
if __name__ == "__main__":
  fix_it()