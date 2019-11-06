#from common.numpy_fast import clip
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.subaru import subarucan
from selfdrive.car.subaru.values import CAR, DBC
from selfdrive.can.packer import CANPacker


class CarControllerParams():
  def __init__(self, car_fingerprint):
    self.STEER_MAX = 2047              # max_steer 4095
    self.STEER_STEP = 2                # how often we update the steer cmd
    self.STEER_DELTA_UP = 70           # torque increase per refresh, 0.68s to max
    self.STEER_DELTA_DOWN = 70         # torque decrease per refresh
    if car_fingerprint == CAR.IMPREZA:
      self.STEER_DRIVER_ALLOWANCE = 60   # allowed driver torque before start limiting
      self.STEER_DRIVER_MULTIPLIER = 10   # weight driver torque heavily
      self.STEER_DRIVER_FACTOR = 1     # from dbc
    if car_fingerprint in (CAR.OUTBACK, CAR.LEGACY):
      self.STEER_DRIVER_ALLOWANCE = 600   # allowed driver torque before start limiting
      self.STEER_DRIVER_MULTIPLIER = 1   # weight driver torque heavily
      self.STEER_DRIVER_FACTOR = 1     # from dbc


class CarController():
  def __init__(self, car_fingerprint):
    self.apply_steer_last = 0
    self.car_fingerprint = car_fingerprint
    self.es_distance_cnt = -1
    self.es_lkas_cnt = -1
    self.counter = 0
    self.button_last = 0
    self.resume_required = 1

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.params = CarControllerParams(car_fingerprint)
    self.packer = CANPacker(DBC[car_fingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, left_line, right_line):
    """ Controls thread """

    P = self.params

    # Send CAN commands.
    can_sends = []

    ### STEER ###

    if (frame % P.STEER_STEP) == 0:
      final_steer = actuators.steer if enabled else 0.
      apply_steer = int(round(final_steer * P.STEER_MAX))

      # limits due to driver torque
      apply_steer = apply_std_steer_torque_limits(apply_steer, self.apply_steer_last, CS.steer_torque_driver, P)

      if not enabled:
        apply_steer = 0.

      if self.car_fingerprint in (CAR.OUTBACK, CAR.LEGACY):

        # add noise to prevent lkas fault from constant torque value for over 1s
        if enabled and apply_steer == self.apply_steer_last:
          self.counter =+ 1
          if self.counter == 50:
            apply_steer = round(int(apply_steer * 0.99))
        else:
          self.counter = 0

      can_sends.append(subarucan.create_steering_control(self.packer, CS.CP.carFingerprint, apply_steer, frame, P.STEER_STEP))

      self.apply_steer_last = apply_steer

    if self.car_fingerprint == CAR.IMPREZA:
      if self.es_distance_cnt != CS.es_distance_msg["Counter"]:
        can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg, pcm_cancel_cmd))
        self.es_distance_cnt = CS.es_distance_msg["Counter"]

      if self.es_lkas_cnt != CS.es_lkas_msg["Counter"]:
        can_sends.append(subarucan.create_es_lkas(self.packer, CS.es_lkas_msg, visual_alert, left_line, right_line))
        self.es_lkas_cnt = CS.es_lkas_msg["Counter"]

    # button control
    if (frame % 5) == 0 and self.car_fingerprint in (CAR.OUTBACK, CAR.LEGACY):
      # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
      fake_button = CS.button

      # cancel and resume when lead car is greater than 1.8m away to remove standstill bit
      if CS.brake_hold and CS.close_distance > 1.2 and enabled:
        fake_button = 1
        self.resume_required = 1
      if self.resume_required == 1 and not enabled and not CS.brake_hold and CS.main_on:
        fake_button = 4
        self.resume_required = 0

      # disengage ACC when OP is disengaged
      if (pcm_cancel_cmd and enabled):
        fake_button = 1
      # turn main on if off
      if not CS.main_on and CS.ready:
        fake_button = 1

      # unstick previous mocked button press
      if self.button_last != 0:
        fake_button = self.button_last
      self.button_last = CS.button

      can_sends.append(subarucan.create_es_throttle_control(self.packer, fake_button, CS.es_accel_msg))

    return can_sends
