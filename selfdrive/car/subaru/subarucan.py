import copy
from cereal import car
from selfdrive.car.subaru.values import CAR

VisualAlert = car.CarControl.HUDControl.VisualAlert

def subaru_checksum(packer, values, addr):
  dat = packer.make_can_msg(addr, 0, values)[2]
  return (sum(dat[1:]) + (addr >> 8) + addr) & 0xff

def subaru_preglobal_checksum(packer, values, addr):
  dat = packer.make_can_msg(addr, 0, values)[2]
  return (sum(dat[:7])) % 256

def create_steering_control(packer, car_fingerprint, apply_steer, frame, steer_step):

  if car_fingerprint == CAR.IMPREZA:
    #counts from 0 to 15 then back to 0 + 16 for enable bit
    idx = ((frame // steer_step) % 16)

    values = {
      "Counter": idx,
      "LKAS_Output": apply_steer,
      "LKAS_Request": 1 if apply_steer != 0 else 0,
      "SET_1": 1
    }
    values["Checksum"] = subaru_checksum(packer, values, 0x122)

  if car_fingerprint in (CAR.OUTBACK, CAR.LEGACY):
    #counts from 0 to 7 then back to 0
    idx = (frame / steer_step) % 8

    values = {
      "Counter": idx,
      "LKAS_Command": apply_steer,
      "LKAS_Active": 1 if apply_steer != 0 else 0
    }
    values["Checksum"] = subaru_preglobal_checksum(packer, values, "ES_LKAS")

  return packer.make_can_msg("ES_LKAS", 0, values)

def create_steering_status(packer, car_fingerprint, apply_steer, frame, steer_step):

  if car_fingerprint == CAR.IMPREZA:
    values = {}
    values["Checksum"] = subaru_checksum(packer, {}, 0x322)

  return packer.make_can_msg("ES_LKAS_State", 0, values)

def create_es_distance(packer, es_distance_msg, pcm_cancel_cmd):

  values = copy.copy(es_distance_msg)
  if pcm_cancel_cmd:
    values["Main"] = 1

  values["Checksum"] = subaru_checksum(packer, values, 545)

  return packer.make_can_msg("ES_Distance", 0, values)

def create_es_lkas(packer, es_lkas_msg, visual_alert, left_line, right_line):

  values = copy.copy(es_lkas_msg)
  if visual_alert == VisualAlert.steerRequired:
    values["Keep_Hands_On_Wheel"] = 1

  values["LKAS_Left_Line_Visible"] = int(left_line)
  values["LKAS_Right_Line_Visible"] = int(right_line)

  values["Checksum"] = subaru_checksum(packer, values, 802)

  return packer.make_can_msg("ES_LKAS_State", 0, values)

def create_brake(packer, frame, enabled, error, brake):
  
  #counts from 0 to 7 then back to 0
  idx = (frame / 5) % 8
  values = {
    "Counter": idx,
    "Brake_Pressure": brake,
    "Brake_Light": 1 if brake > 0 else 0,
    "ES_Error": error,
    "Brake_On": 1 if brake > 0 else 0,
    "Cruise_Activated": enabled,
  }
  values["Checksum"] = subaru_preglobal_checksum(packer, values, "ES_Brake")

  return packer.make_can_msg("ES_Brake", 0, values)


def create_es_throttle_control(packer, frame, enabled, es_throttle, fake_button, throttle, brake):

  #counts from 0 to 7 then back to 0
  idx = (frame / 5) % 8

  values = {
    "Throttle_Cruise": throttle,
    "Button": fake_button,
    "Brake_On": 1 if brake > 0 else 0,
    "NEW_SIGNAL_1": 0,
    "Standstill": 0,
    "Standstill_2": 0,
    "SET_1": 0,
    "CloseDistance": 5,
    "Counter": idx,

  }
  values["Checksum"] = subaru_preglobal_checksum(packer, values, "ES_CruiseThrottle")

  return packer.make_can_msg("ES_CruiseThrottle", 0, values)


def create_es_rpm_control(packer, frame, enabled, brake, rpm):
  
  #counts from 0 to 7 then back to 0
  idx = (frame / 5) % 8

  values = {
    "Brake": 1 if brake > 0 else 0,
    "Cruise_Activated": enabled,
    "RPM": rpm,
    "Counter": idx,
  }
  values["Checksum"] = subaru_preglobal_checksum(packer, values, "ES_RPM")

  return packer.make_can_msg("ES_RPM", 0, values)


def create_es_dash_control(packer, frame, enabled, part_1, part_2, part_3, part_4, error, v_cruise_pcm, ready, brake):
  
  #counts from 0 to 7 then back to 0
  idx = (frame / 5) % 8
  
  values = {
    "Cruise_On": 1,
    "Cruise_On_2": 1,
    "Cruise_Activated": enabled,
    "Distance_Bars": 1,
    "Cruise_Set_Speed": v_cruise_pcm,
    "Lead_Car": lead_car,
    "NEW_SIGNAL_1": 1 if brake > 0 else 0,
    "Counter": idx,
    "Obstacle_Distance": 5,
  }
  return packer.make_can_msg("ES_DashStatus", 0, values)
