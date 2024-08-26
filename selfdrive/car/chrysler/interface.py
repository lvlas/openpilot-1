#!/usr/bin/env python3
from cereal import car
from panda import Panda
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.chrysler.values import CAR, DBC, RAM_HD, RAM_DT, RAM_CARS, HYBRID_CARS, ChryslerFlags
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from common.params import Params
from common.cached_params import CachedParams

params = Params()
cachedParams = CachedParams()
ButtonType = car.CarState.ButtonEvent.Type

class CarInterface(CarInterfaceBase):
  ACCEL_MAX = 2.  # m/s2, high to not limit stock ACC
  ACCEL_MIN = -3.5  # m/s2
  @staticmethod
  def get_pid_accel_limits(CS, CP, current_speed, cruise_speed):
    return CarInterface.ACCEL_MIN, CarInterface.accel_max(CS)

  @staticmethod
  def accel_max(CS):
    maxAccel = CarInterface.ACCEL_MAX
    if CS.longControl:
      eco = cachedParams.get_float('jvePilot.carState.accEco', 1000)
      if eco == 1:
        maxAccel = cachedParams.get_float('jvePilot.settings.accEco.longAccelLevel1', 1000)
      elif eco == 2:
        maxAccel = cachedParams.get_float('jvePilot.settings.accEco.longAccelLevel2', 1000)
      else:
        maxAccel = 2

    return maxAccel

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "chrysler"
    ret.dashcamOnly = candidate in RAM_HD

    # radar parsing needs some work, see https://github.com/commaai/openpilot/issues/26842
    ret.radarUnavailable = False # DBC[candidate]['radar'] is None
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4

    # safety config
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.chrysler)]
    if candidate in RAM_HD:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_HD
    elif candidate in RAM_DT:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_DT

    #CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    #if candidate not in RAM_CARS:
    #  # Newer FW versions standard on the following platforms, or flashed by a dealer onto older platforms have a higher minimum steering speed.
    #  new_eps_platform = candidate in (CAR.CHRYSLER_PACIFICA_2019_HYBRID, CAR.CHRYSLER_PACIFICA_2020, CAR.JEEP_GRAND_CHEROKEE_2019, CAR.DODGE_DURANGO)
    #  new_eps_firmware = any(fw.ecu == 'eps' and fw.fwVersion[:4] >= b"6841" for fw in car_fw)
    #  if new_eps_platform or new_eps_firmware:
    #    ret.flags |= ChryslerFlags.HIGHER_MIN_STEERING_SPEED.value
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.2)
    ret.minSteerSpeed = 3.8  # m/s

    #ret.lateralTuning.pid.kpBP = [0., 10., 35.]
#    ret.lateralTuning.pid.kpV = [0.02, 0.02, 0.02]

#    ret.lateralTuning.pid.kiBP = [0., 15., 30.]
#    ret.lateralTuning.pid.kiV = [0.003, 0.003, 0.004]

#    ret.lateralTuning.pid.kf = 0.00002   # full torque for 10 deg at 80mph means 0.00007818594

    ret.experimentalLongitudinalAvailable = True #Params().get_bool('ChryslerMangoLong')
    ret.openpilotLongitudinalControl = True #Params().get_bool('ChryslerMangoLong')

    # Long tuning Params -  make individual params for cars, baseline Pacifica Hybrid
    ret.longitudinalTuning.kpBP = [0., 6., 10., 35.]
    ret.longitudinalTuning.kpV = [.4, .6, 0.5, .2]
    ret.longitudinalTuning.kiBP = [0., 30.]
    ret.longitudinalTuning.kiV = [.001, .001]
    ret.stoppingControl = True
    ret.stoppingDecelRate = 0.2
    
    # Chrysler
    #if candidate in (CAR.CHRYSLER_PACIFICA_2017_HYBRID, CAR.CHRYSLER_PACIFICA_2018, CAR.CHRYSLER_PACIFICA_2018_HYBRID, \
    #                 CAR.CHRYSLER_PACIFICA_2019_HYBRID, CAR.CHRYSLER_PACIFICA_2020, CAR.DODGE_DURANGO):
    #  ret.lateralTuning.init('pid')
    #  ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
    #  ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
    #  ret.lateralTuning.pid.kf = 0.00006
    if candidate in (CAR.CHRYSLER_PACIFICA_2017_HYBRID, CAR.CHRYSLER_PACIFICA_2018, CAR.CHRYSLER_PACIFICA_2018_HYBRID, CAR.CHRYSLER_PACIFICA_2019_HYBRID, CAR.CHRYSLER_PACIFICA_2020):
      ret.minSteerSpeed = 0.0 #17.5  if not Params().get_bool('ChryslerMangoLat') and not Params().get_bool('LkasFullRangeAvailable') else 0 # m/s 17 on the way up, 13 on the way down once engaged.
      ret.steerActuatorDelay = 0.2
      #ret.experimentalLongitudinalAvailable = candidate not in HYBRID_CARS

    # Jeep
    elif candidate in (CAR.JEEP_GRAND_CHEROKEE, CAR.JEEP_GRAND_CHEROKEE_2019):
      ret.steerActuatorDelay = 0.2

      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
      ret.lateralTuning.pid.kf = 0.00006

      ret.enableBsm = True
      ret.experimentalLongitudinalAvailable = True

    # Ram
    elif candidate == CAR.RAM_1500_5TH_GEN:
      ret.steerActuatorDelay = 0.2
      ret.wheelbase = 3.88
      # Older EPS FW allow steer to zero
      if any(fw.ecu == 'eps' and b"68" < fw.fwVersion[:4] <= b"6831" for fw in car_fw):
        ret.minSteerSpeed = 0.

    elif candidate == CAR.RAM_HD_5TH_GEN:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, 1.0, False)

    else:
      raise ValueError(f"Unsupported car: {candidate}")

    #if ret.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED:
    #  # TODO: allow these cars to steer down to 13 m/s if already engaged.
    #  # TODO: Durango 2020 may be able to steer to zero once above 38 kph
    #  ret.minSteerSpeed = 17.5  # m/s 17 on the way up, 13 on the way down once engaged.

    ret.centerToFront = ret.wheelbase * 0.44
    ret.enableBsm |= 720 in fingerprint[0]
    ret.enablehybridEcu = 655 in fingerprint[0] or 291 in fingerprint[0]    

    ret.openpilotLongitudinalControl = True  # kind of...
    ret.pcmCruiseSpeed = False  # Let jvePilot control the pcm cruise speed

    # Autodetect WP
    if (0x4FF in fingerprint[0]) or params.get_bool("jvePilot.settings.steer.noMinimum"):
      params.put_bool_nonblocking("jvePilot.settings.steer.noMinimum", True)
      ret.minSteerSpeed = -0.1

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    #ret.buttonEvents = create_button_events(self.CS.distance_button, self.CS.prev_distance_button, {1: ButtonType.gapAdjustCruise})

    ret.steerFaultPermanent = self.CC.steerErrorMod
    ret.hightorqUnavailable = self.CC.hightorqUnavailable

    # events
    events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low])

    # Low speed steer alert hysteresis logic
    if self.CP.minSteerSpeed > 0. and ret.vEgo < (self.CP.minSteerSpeed + 0.5):
      self.low_speed_alert = True
    elif ret.vEgo > (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = False
    if self.low_speed_alert:

    if 0: #ret.vEgo < self.CP.minSteerSpeed and not Params().get_bool('ChryslerMangoLat') and not Params().get_bool('LkasFullRangeAvailable'):
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    if self.CC.acc_enabled and (self.CS.accbrakeFaulted or self.CS.accengFaulted):
      events.add(car.CarEvent.EventName.accFaulted)

    ret.events = events.to_msg()

    return ret
