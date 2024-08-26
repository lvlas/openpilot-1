from numpy import interp
from openpilot.selfdrive.car.interfaces import GearShifter
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car.chrysler.values import CAR, CarControllerParams, ChryslerFlags

import math
from common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_meas_steer_torque_limits, button_pressed
from openpilot.selfdrive.car.chrysler import chryslercan
from openpilot.selfdrive.car.chrysler.values import RAM_CARS, CarControllerParams, ChryslerFlags, DRIVE_PERSONALITY
from openpilot.selfdrive.car.interfaces import CarControllerBase

from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MIN, V_CRUISE_MIN_IMPERIAL
from openpilot.selfdrive.car.chrysler.long_carcontroller_v1 import LongCarControllerV1
from common.conversions import Conversions as CV
from common.cached_params import CachedParams
from common.params import Params
from cereal import car, messaging

GearShifter = car.CarState.GearShifter
ButtonType = car.CarState.ButtonEvent.Type

V_CRUISE_MIN_IMPERIAL_MS = V_CRUISE_MIN_IMPERIAL * CV.KPH_TO_MS
V_CRUISE_MIN_MS = V_CRUISE_MIN * CV.KPH_TO_MS
AUTO_FOLLOW_LOCK_MS = 3 * CV.MPH_TO_MS
EXTEND_FUTURE_MAX = 10 * CV.MPH_TO_MS

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0                                                                         #jeste nechavam
    self.ccframe = 0

    self.hud_count = 0
    self.next_lkas_control_change = 0
    #self.lkas_control_bit_prev = False
    self.last_button_frame = 0                                                             #jeste nechavam

    self.car_fingerprint = CP.carFingerprint
    self.gone_fast_yet = False
    self.steer_rate_limited = False
    self.timer = 0
    self.steerErrorMod = False
    self.steer_type = int(0)
    self.hightorqUnavailable = False
    self.acc_stop_timer = 0
    self.stop_button_spam = 0
    self.wheel_button_counter_prev = 0
    self.lead_dist_at_stop = 0
    self.hybridEcu = CP.enablehybridEcu
    self.mango_lat_active = True #Params().get_bool('ChryslerMangoLat')
    self.full_range_steer = False #Params().get_bool('LkasFullRangeAvailable')
    self.mango_mode_active = self.mango_lat_active or self.full_range_steer

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)

    self.sm = messaging.SubMaster(['longitudinalPlan'])
    self.settingsParams = Params()
    self.cachedParams = CachedParams()
    self.minAccSetting = V_CRUISE_MIN_MS if self.settingsParams.get_bool("IsMetric") else V_CRUISE_MIN_IMPERIAL_MS
    self.round_to_unit = CV.MS_TO_KPH if self.settingsParams.get_bool("IsMetric") else CV.MS_TO_MPH
    self.steerNoMinimum = CP.minSteerSpeed < 0
    self.auto_enable_acc = self.settingsParams.get_bool("jvePilot.settings.autoEnableAcc")

    self.autoFollowDistanceLock = None
    self.button_frame = 0
    self.last_target = 0
    self.last_personality = None
    self.low_steer = not self.CP.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED
    self.steer_gap = 0.5 if self.CP.carFingerprint in RAM_CARS else 3.0

    self.long_controller = LongCarControllerV1(self.CP, self.params, self.packer)

  def update(self, CC, CS, now_nanos):
    #can_sends = []
    self.sm.update(0)



    # *** compute control surfaces ***
    enabled = CC.enabled
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    wp_type = int(0)
    self.hightorqUnavailable = False

    if self.full_range_steer:
      wp_type = int(1)
    if self.mango_lat_active:
      wp_type = int(2)

    if enabled:
      if self.timer < 99 and wp_type == 1 and CS.out.vEgo < 65:
        self.timer += 1
      else:
        self.timer = 99
    else:
      self.timer = 0

    lkas_active = self.timer == 99 and  (self.ccframe >= 500)

    # steer torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)

    if not self.mango_mode_active:
      moving_fast = CS.out.vEgo > self.CP.minSteerSpeed  # for status message
      if CS.out.vEgo > (self.CP.minSteerSpeed - 0.5):  # for command high bit
        self.gone_fast_yet = True
      elif self.car_fingerprint in (CAR.CHRYSLER_PACIFICA_2019_HYBRID, CAR.CHRYSLER_PACIFICA_2020, CAR.JEEP_GRAND_CHEROKEE_2019):
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          self.gone_fast_yet = False  # < 14.5m/s stock turns off this bit, but fine down to 13.5
      lkas_active = moving_fast and enabled

      if not lkas_active:
        apply_steer = 0

    self.steer_rate_limited = new_steer != apply_steer
    self.apply_steer_last = apply_steer
    self.steer_type = wp_type

    if wp_type != 2:
      self.steerErrorMod = CS.steerError
      if self.steerErrorMod:
        self.steer_type = int(0)
    elif CS.apaFault or CS.out.gearShifter not in (GearShifter.drive, GearShifter.low) or \
            not CS.veh_on or CS.apa_steer_status:
      self.steer_type = int(0)

    if (self.ccframe < 500) or \
            (self.steer_type == int(0) and CS.out.gearShifter in (GearShifter.drive, GearShifter.low) and not CS.apaFault and self.mango_lat_active):
      self.hightorqUnavailable = True

    self.apaActive = CS.apasteerOn and self.steer_type == 2

    can_sends = []

    #*** control msgs ***

    self.resume_press = False
    if CS.acc_hold and CS.out.standstill:
      self.acc_stop_timer += 1
      if self.acc_stop_timer > 100: # send resume spam at 1.8 sec; looks like ACC auto resumes by itself if lead moves within 2 seconds
        self.resume_press = True
    else:
      self.acc_stop_timer = 0
      self.lead_dist_at_stop = CS.lead_dist

    if CS.acc_button_pressed:
      self.stop_button_spam = self.ccframe + 50 # stop spamming for 500msec if driver pressed any acc steering wheel button

    wheel_button_counter_change = CS.wheel_button_counter != self.wheel_button_counter_prev
    if wheel_button_counter_change:
      self.wheel_button_counter_prev = CS.wheel_button_counter

    self.op_cancel_cmd = False

    if (self.ccframe % 8 < 4) and self.ccframe >= self.stop_button_spam:  #and wheel_button_counter_change
      button_type = None
      if not enabled and pcm_cancel_cmd and CS.out.cruiseState.enabled and not self.op_long_enable:
        button_type = 'ACC_CANCEL'
        self.op_cancel_cmd = True
      elif enabled and self.resume_press and not self.op_long_enable and ((CS.lead_dist > self.lead_dist_at_stop) or (CC.hudControl.leadvRel > 0) or (15 > CS.lead_dist >= 6.)):
        button_type = 'ACC_RESUME'
      elif (enabled and CS.out.standstill):
        button_type = 'ACC_RESUME'

      if button_type is not None:
        new_msg = create_wheel_buttons(self.packer, CS.wheel_button_counter + 1, button_type)
        can_sends.append(new_msg)

    # LKAS_HEARTBIT is forwarded by Panda so no need to send it here.
    # frame is 100Hz (0.01s period)
    if (self.ccframe % 2 == 0) and wp_type == 2:  # 0.02s period
      new_msg = create_mango_hud(
          self.packer, self.apaActive, CS.apaFault, lkas_active,
          self.steer_type)
      can_sends.append(new_msg)

    if (self.ccframe % 2 == 0) and wp_type != 2:  # 0.25s period
      new_msg = create_lkas_hud(
          self.packer, CS.out.gearShifter, lkas_active,
          self.hud_count, self.steer_type)
      can_sends.append(new_msg)

    if self.ccframe % 25 == 0:
      self.hud_count += 1

    new_msg = create_lkas_command(self.packer, int(apply_steer), lkas_active, CS.lkas_counter)
    can_sends.append(new_msg)
    
    
    # cruise buttons
    #das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

    # ACC cancellation
    # if CC.cruiseControl.cancel:
    #   self.last_button_frame = self.frame
    #   can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, cancel=True))
    #
    # # ACC resume from standstill
    # elif CC.cruiseControl.resume:
    #   self.last_button_frame = self.frame
    #   can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, resume=True))

    # jvePilot
    if button_pressed(CS.out, ButtonType.lkasToggle, False):
      CS.lkas_button_light = not CS.lkas_button_light
      self.settingsParams.put_nonblocking("jvePilot.settings.lkasButtonLight", "1" if CS.lkas_button_light else "0")
    if self.frame % 10 == 0:
      lkas_disabled = CS.lkas_button_light or CS.out.steerFaultPermanent
      new_msg = chryslercan.create_lkas_heartbit(self.packer, lkas_disabled, CS.lkasHeartbit)
      can_sends.append(new_msg)
    self.wheel_button_control(CC, CS, can_sends, CC.enabled, das_bus, CC.cruiseControl.cancel, CC.cruiseControl.resume)

    # HUD alerts
#    if self.frame % 25 == 0:
#      if CS.lkas_car_model != -1:
#        can_sends.append(chryslercan.create_lkas_hud(self.packer, self.CP, CC.latActive and self.lkas_control_bit_prev, CC.hudControl.visualAlert,
#                                                     self.hud_count, CS.lkas_car_model, CS.auto_high_beam,
#                                                     CC.enabled or CC.jvePilotState.carControl.aolcAvailable, CS.out.cruiseState.available))
#        self.hud_count += 1

    # steering
#    new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
#    if self.frame % self.params.STEER_STEP == 0 or abs(new_steer - int(self.apply_steer_last) > self.cachedParams.get_float('jvePilot.settings.steer.chillLevel', 1000)):
#      lkas_control_bit = self.lkas_control_bit_prev
#      if CS.out.vEgo > self.CP.minSteerSpeed or self.steerNoMinimum:
#        lkas_control_bit = CC.latActive
#      elif CS.out.vEgo < (self.CP.minSteerSpeed - self.steer_gap):
#        lkas_control_bit = False
#
#      if self.low_steer and self.lkas_control_bit_prev:
#        # low steer vehicles never turn this off
#        lkas_control_bit = True
#      else:
#        # EPS faults if LKAS enables too quickly
#        if lkas_control_bit and self.lkas_control_bit_prev != lkas_control_bit:
#          if self.next_lkas_control_change == 0:
#            self.next_lkas_control_change = self.frame + 70
#        else:
#          self.next_lkas_control_change = 0
#        lkas_control_bit = lkas_control_bit and (self.frame > self.next_lkas_control_change)
#
#      self.lkas_control_bit_prev = lkas_control_bit
#
#      apply_steer = 0
#      if CC.latActive and lkas_control_bit:
#        apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)
#
#      self.apply_steer_last = apply_steer
#
#      can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit, self.steerNoMinimum, CC.latActive))

    if CC.enabled:
      # auto set profile
      follow_distance = CC.jvePilotState.carState.accFollowDistance or 0
      acc_eco = CC.jvePilotState.carControl.accEco or 0
      personality = acc_eco if CS.longControl else DRIVE_PERSONALITY[acc_eco][follow_distance]
      if personality != self.last_personality:
        self.last_personality = personality
        self.settingsParams.put_nonblocking('LongitudinalPersonality', str(personality))

    self.long_controller.acc(self.sm['longitudinalPlan'], self.frame, CC, CS, can_sends)

    self.frame += 1
    self.ccframe += 1    

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    return new_actuators, can_sends

  def wheel_button_control(self, CC, CS, can_sends, enabled, das_bus, cancel, resume):
    button_counter = CS.button_counter
    if button_counter == self.last_button_frame:
      return
    self.last_button_frame = button_counter

    if not self.long_controller.button_control(CC, CS):
      self.button_frame += 1
      button_counter_offset = 1
      buttons_to_press = []
      if cancel:
        buttons_to_press = ['ACC_Cancel']
      elif not button_pressed(CS.out, ButtonType.cancel):
        if enabled and not CS.out.brakePressed:
          button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
          if button_counter_offset is not None:
            if resume:
              buttons_to_press = ["ACC_Resume"]
            elif CS.out.cruiseState.enabled:  # Control ACC
              buttons_to_press = [self.auto_follow_button(CC, CS), self.hybrid_acc_button(CC, CS)]

      # ACC Auto enable
      if self.auto_enable_acc and self.frame < 50:
        if not CS.out.cruiseState.available:
          buttons_to_press.append("ACC_OnOff")
        else:
          self.auto_enable_acc = False

      buttons_to_press = list(filter(None, buttons_to_press))
      if buttons_to_press is not None and len(buttons_to_press) > 0:
        new_msg = chryslercan.create_wheel_buttons_command(self.packer, das_bus, button_counter + button_counter_offset, buttons_to_press)
        can_sends.append(new_msg)

  def hybrid_acc_button(self, CC, CS):
    # Move the adaptive curse control to the target speed
    eco_limit = None
    if CC.jvePilotState.carControl.accEco == 1:
      eco_limit = self.cachedParams.get_float('jvePilot.settings.accEco.speedAheadLevel1', 1000)
    elif CC.jvePilotState.carControl.accEco == 2:
      eco_limit = self.cachedParams.get_float('jvePilot.settings.accEco.speedAheadLevel2', 1000)

    if len(self.sm['longitudinalPlan'].speeds):
      extendFuture = clip(min(self.sm['longitudinalPlan'].accels) * 2, -EXTEND_FUTURE_MAX, EXTEND_FUTURE_MAX)
      targetFuture = self.sm['longitudinalPlan'].speeds[-1] + extendFuture
    else:
      targetFuture = 0

    target = self.acc_hysteresis(targetFuture)
    if eco_limit:
      target = min(target, CS.out.vEgo + (eco_limit * CV.MPH_TO_MS))

    target = math.ceil(min(CC.jvePilotState.carControl.vMaxCruise, target) * self.round_to_unit)
    current = round(CS.out.cruiseState.speed * self.round_to_unit)
    minSetting = round(self.minAccSetting * self.round_to_unit)

    if target < current and current > minSetting:
      return 'ACC_Decel'
    elif target > current:
      return 'ACC_Accel'

  def auto_follow_button(self, CC, CS):
    if CC.jvePilotState.carControl.autoFollow:
      crossover = [0,
                   self.cachedParams.get_float('jvePilot.settings.autoFollow.speed1-2Bars', 1000) * CV.MPH_TO_MS,
                   self.cachedParams.get_float('jvePilot.settings.autoFollow.speed2-3Bars', 1000) * CV.MPH_TO_MS,
                   self.cachedParams.get_float('jvePilot.settings.autoFollow.speed3-4Bars', 1000) * CV.MPH_TO_MS]

      if CS.out.vEgo < crossover[1]:
        target_follow = 0
      elif CS.out.vEgo < crossover[2]:
        target_follow = 1
      elif CS.out.vEgo < crossover[3]:
        target_follow = 2
      else:
        target_follow = 3

      if self.autoFollowDistanceLock is not None and abs(crossover[self.autoFollowDistanceLock] - CS.out.vEgo) > AUTO_FOLLOW_LOCK_MS:
        self.autoFollowDistanceLock = None  # unlock

      if CC.jvePilotState.carState.accFollowDistance != target_follow and (self.autoFollowDistanceLock or target_follow) == target_follow:
        self.autoFollowDistanceLock = target_follow  # going from close to far, use upperbound

        if CC.jvePilotState.carState.accFollowDistance > target_follow:
          return 'ACC_Distance_Dec'
        else:
          return 'ACC_Distance_Inc'

  def acc_hysteresis(self, new_target):
    if new_target > self.last_target:
      self.last_target = new_target
    elif new_target < self.last_target - 0.75 * CV.MPH_TO_MS:
      self.last_target = new_target

    return self.last_target

