from cereal import car
from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.chrysler.values import DBC, STEER_THRESHOLD, RAM_CARS, CAR
from common.cached_params import CachedParams

import numpy as np

ButtonType = car.CarState.ButtonEvent.Type

CHECK_BUTTONS = {ButtonType.cancel: ["CRUISE_BUTTONS", 'ACC_Cancel'],
                 ButtonType.resumeCruise: ["CRUISE_BUTTONS", 'ACC_Resume'],
                 ButtonType.accelCruise: ["CRUISE_BUTTONS", 'ACC_Accel'],
                 ButtonType.decelCruise: ["CRUISE_BUTTONS", 'ACC_Decel'],
                 ButtonType.followInc: ["CRUISE_BUTTONS", 'ACC_Distance_Inc'],
                 ButtonType.followDec: ["CRUISE_BUTTONS", 'ACC_Distance_Dec'],
                 ButtonType.lkasToggle: ["TRACTION_BUTTON", 'TOGGLE_LKAS']}

PEDAL_GAS_PRESSED_XP = [0, 32, 255]
PEDAL_BRAKE_PRESSED_XP = [0, 24, 255]
PEDAL_PRESSED_YP = [0, 128, 255]

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    self.auto_high_beam = 0
    self.button_counter = 0
    self.lkas_car_model = -1

    if CP.carFingerprint in RAM_CARS:
      self.shifter_values = can_define.dv["Transmission_Status"]["Gear_State"]
    else:
      self.shifter_values = can_define.dv["GEAR"]["PRNDL"]

    self.lkasHeartbit = None

    # long control
    self.longControl = False
    self.cachedParams = CachedParams()
    self.das_3 = None
    self.longEnabled = False
    self.longControl = False
    self.gasRpm = None
    self.allowLong = CP.carFingerprint in (CAR.JEEP_CHEROKEE, CAR.JEEP_CHEROKEE_2019)
    self.torqMin = None
    self.torqMax = None
    self.currentGear = None

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    # lock info
    ret.doorOpen = any([cp.vl["BCM_1"]["DOOR_OPEN_FL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_FR"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["ORC_1"]["SEATBELT_DRIVER_UNLATCHED"] == 1

    # brake pedal
    ret.brake = 0
    ret.brakePressed = cp.vl["ESP_1"]['Brake_Pedal_State'] == 1  # Physical brake pedal switch

    # gas pedal
    ret.gas = cp.vl["ECM_5"]["Accelerator_Position"]
    ret.gasPressed = ret.gas > 1e-5

    # car speed
    if self.CP.carFingerprint in RAM_CARS:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["Transmission_Status"]["Gear_State"], None))
    else:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["GEAR"]["PRNDL"], None))
    ret.vEgoRaw = cp.vl["ESP_8"]["Vehicle_Speed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw <= 0.001
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["ESP_6"]["WHEEL_SPEED_FL"],
      cp.vl["ESP_6"]["WHEEL_SPEED_FR"],
      cp.vl["ESP_6"]["WHEEL_SPEED_RL"],
      cp.vl["ESP_6"]["WHEEL_SPEED_RR"],
      unit=1,
    )

    # button presses
    ret.leftBlinker = cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 2
    ret.genericToggle = cp.vl["STEERING_LEVERS"]["HIGH_BEAM_PRESSED"] == 1

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE"] + cp.vl["STEERING"]["STEERING_ANGLE_HP"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl["EPS_2"]["COLUMN_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["EPS_TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # cruise state
    cp_cruise = cp_cam if self.CP.carFingerprint in RAM_CARS else cp

    self.longControl = self.allowLong and cp.vl["DAS_4"]["ACC_STATE"] == 0 and self.cachedParams.get_bool('jvePilot.settings.longControl', 1000)
    if self.longControl:
      ret.jvePilotCarState.longControl = True
      ret.cruiseState.enabled = self.longEnabled
      ret.cruiseState.available = True
      ret.cruiseState.nonAdaptive = False
      ret.cruiseState.standstill = False
      ret.accFaulted = False
      self.torqMin = cp.vl["DAS_3"]["ENGINE_TORQUE_REQUEST"]
      self.torqMax = cp.vl["ECM_TRQ"]["ENGINE_TORQ_MAX"]
      self.currentGear = cp.vl['TCM_A7']["CurrentGear"]
      self.gasRpm = cp.vl["ECM_1"]["ENGINE_RPM"]
      self.das_3 = cp.vl['DAS_3']
    else:
      ret.jvePilotCarState.longControl = False
      self.longEnabled = False
      ret.cruiseState.available = cp_cruise.vl["DAS_3"]["ACC_AVAILABLE"] == 1
      ret.cruiseState.enabled = cp_cruise.vl["DAS_3"]["ACC_ACTIVE"] == 1
      ret.cruiseState.nonAdaptive = cp_cruise.vl["DAS_4"]["ACC_STATE"] in (1, 2)  # 1 NormalCCOn and 2 NormalCCSet
      ret.cruiseState.standstill = cp_cruise.vl["DAS_3"]["ACC_STANDSTILL"] == 1
      ret.accFaulted = cp_cruise.vl["DAS_3"]["ACC_FAULTED"] != 0

    ret.cruiseState.speed = cp_cruise.vl["DAS_4"]["ACC_SET_SPEED_KPH"] * CV.KPH_TO_MS
    self.lkasHeartbit = cp_cam.vl["LKAS_HEARTBIT"]

    if self.CP.carFingerprint in RAM_CARS:
      self.auto_high_beam = cp_cam.vl["DAS_6"]['AUTO_HIGH_BEAM_ON']  # Auto High Beam isn't Located in this message on chrysler or jeep currently located in 729 message
      ret.steerFaultTemporary  = cp.vl["EPS_3"]["DASM_FAULT"] == 1
    else:
      ret.steerFaultPermanent = cp.vl["EPS_2"]["LKAS_STATE"] == 4

    # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BSM_1"]["LEFT_STATUS"] == 1
      ret.rightBlindspot = cp.vl["BSM_1"]["RIGHT_STATUS"] == 1

    self.lkas_car_model = cp_cam.vl["DAS_6"]["CAR_MODEL"]
    self.button_counter = cp.vl["CRUISE_BUTTONS"]["COUNTER"]

    brake = cp.vl["ESP_8"]["BRK_PRESSURE"]
    gas = cp.vl["ACCEL_RELATED_120"]["ACCEL"]
    if gas > 0:
      ret.jvePilotCarState.pedalPressedAmount = float(np.interp(gas, PEDAL_GAS_PRESSED_XP, PEDAL_PRESSED_YP)) / 256
    elif brake > 0:
      ret.jvePilotCarState.pedalPressedAmount = float(np.interp(brake / 16, PEDAL_BRAKE_PRESSED_XP, PEDAL_PRESSED_YP)) / -256
    else:
      ret.jvePilotCarState.pedalPressedAmount = 0

    ret.jvePilotCarState.accFollowDistance = int(min(3, max(0, cp.vl["DAS_4"]['ACC_DISTANCE_CONFIG_2'])))

    button_events = []
    for buttonType in CHECK_BUTTONS:
      self.check_button(button_events, buttonType, bool(cp.vl[CHECK_BUTTONS[buttonType][0]][CHECK_BUTTONS[buttonType][1]]))
    ret.buttonEvents = button_events

    return ret

  def check_button(self, button_events, button_type, pressed):
    pressed_frames = 0
    pressed_changed = False
    for ob in self.out.buttonEvents:
      if ob.type == button_type:
        pressed_frames = ob.pressedFrames
        pressed_changed = ob.pressed != pressed
        break

    if pressed or pressed_changed:
      be = car.CarState.ButtonEvent.new_message()
      be.type = button_type
      be.pressed = pressed
      be.pressedFrames = pressed_frames

      if not pressed_changed:
        be.pressedFrames += 1

      button_events.append(be)

  def button_pressed(self, button_type, pressed=True):
    for b in self.out.buttonEvents:
      if b.type == button_type:
        if b.pressed == pressed:
          return b
        break

  @staticmethod
  def get_cruise_signals():
    signals = [
      ("ACC_AVAILABLE", "DAS_3"),
      ("ACC_ACTIVE", "DAS_3"),
      ("ACC_FAULTED", "DAS_3"),
      ("ACC_STANDSTILL", "DAS_3"),
      ("COUNTER", "DAS_3"),
      ("ACC_SET_SPEED_KPH", "DAS_4"),
      ("ACC_STATE", "DAS_4"),
      ("ACC_DISTANCE_CONFIG_2", "DAS_4"),

      ("ACC_GO", "DAS_3", 0),
      ("ENGINE_TORQUE_REQUEST", "DAS_3", 0),
      ("ENGINE_TORQUE_REQUEST_MAX", "DAS_3", 0),
      ("ACC_DECEL", "DAS_3", 0),
      ("ACC_DECEL_REQ", "DAS_3", 0),
      ("ACC_AVAILABLE", "DAS_3", 0),
      ("DISABLE_FUEL_SHUTOFF", "DAS_3", 0),
      ("GR_MAX_REQ", "DAS_3", 0),
      ("STS", "DAS_3", 0),
      ("COLLISION_BRK_PREP", "DAS_3", 0),
      ("ACC_BRK_PREP", "DAS_3", 0),
      ("DISPLAY_REQ", "DAS_3", 0),
      ("COUNTER", "DAS_3", 0),
      ("CHECKSUM", "DAS_3", 0),
    ]
    checks = [
      ("DAS_3", 50),
      ("DAS_4", 50),
    ]
    return signals, checks

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("DOOR_OPEN_FL", "BCM_1"),
      ("DOOR_OPEN_FR", "BCM_1"),
      ("DOOR_OPEN_RL", "BCM_1"),
      ("DOOR_OPEN_RR", "BCM_1"),
      ("Brake_Pedal_State", "ESP_1"),
      ("Accelerator_Position", "ECM_5"),
      ("WHEEL_SPEED_FL", "ESP_6"),
      ("WHEEL_SPEED_RR", "ESP_6"),
      ("WHEEL_SPEED_RL", "ESP_6"),
      ("WHEEL_SPEED_FR", "ESP_6"),
      ("STEERING_ANGLE", "STEERING"),
      ("STEERING_ANGLE_HP", "STEERING"),
      ("STEERING_RATE", "STEERING"),
      ("TURN_SIGNALS", "STEERING_LEVERS"),
      ("HIGH_BEAM_PRESSED", "STEERING_LEVERS"),
      ("SEATBELT_DRIVER_UNLATCHED", "ORC_1"),
      ("COUNTER", "EPS_2",),
      ("COLUMN_TORQUE", "EPS_2"),
      ("EPS_TORQUE_MOTOR", "EPS_2"),
      ("LKAS_STATE", "EPS_2"),
      ("COUNTER", "CRUISE_BUTTONS"),
      ("ACC_Resume", "CRUISE_BUTTONS"),
      ("ACC_Cancel", "CRUISE_BUTTONS"),
      ("ACC_Accel", "CRUISE_BUTTONS"),
      ("ACC_Decel", "CRUISE_BUTTONS"),
      ("ACC_Distance_Inc", "CRUISE_BUTTONS"),
      ("ACC_Distance_Dec", "CRUISE_BUTTONS"),
      ("TOGGLE_LKAS", "TRACTION_BUTTON"),
      ("Vehicle_Speed", "ESP_8"),
      ("ACCEL", "ACCEL_RELATED_120"),
      ("BRK_PRESSURE", "ESP_8"),

      ("ENGINE_RPM", "ECM_1", 0),
      ("ENGINE_TORQ_MIN", "ECM_TRQ", 0),
      ("ENGINE_TORQ_MAX", "ECM_TRQ", 0),
      ("CurrentGear", "TCM_A7", 0),
    ]

    checks = [
      # sig_address, frequency
      ("ESP_1", 50),
      ("EPS_2", 100),
      ("ESP_6", 50),
      ("STEERING", 100),
      ("ECM_5", 50),
      ("CRUISE_BUTTONS", 50),
      ("STEERING_LEVERS", 10),
      ("ORC_1", 2),
      ("BCM_1", 1),
      ("ESP_8", 50),
      ("ACCEL_RELATED_120", 50),
      ("TRACTION_BUTTON", 1),

      ("ECM_1", 50),
      ("ECM_TRQ", 50),
      ("TCM_A7", 50),
    ]

    if CP.enableBsm:
      signals += [
        ("RIGHT_STATUS", "BSM_1"),
        ("LEFT_STATUS", "BSM_1"),
      ]
      checks.append(("BSM_1", 2))

    if CP.carFingerprint in RAM_CARS:
      signals += [
        ("DASM_FAULT", "EPS_3"),
        ("Gear_State", "Transmission_Status"),
      ]
      checks += [
        ("EPS_3", 50),
        ("Transmission_Status", 50),
      ]
    else:
      signals += [
        ("PRNDL", "GEAR"),
      ]
      checks += [
        ("GEAR", 50),
      ]
      signals += CarState.get_cruise_signals()[0]
      checks += CarState.get_cruise_signals()[1]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("CAR_MODEL", "DAS_6"),
    ]
    checks = [
      ("DAS_6", 4),
    ]

    if CP.carFingerprint in RAM_CARS:
      signals += [
        ("AUTO_HIGH_BEAM_ON", "DAS_6"),
      ]
      signals += CarState.get_cruise_signals()[0]
      checks += CarState.get_cruise_signals()[1]
    else:
      # LKAS_HEARTBIT data needs to be forwarded!
      forward_lkas_heartbit_signals = [
        ("AUTO_HIGH_BEAM", "LKAS_HEARTBIT"),
        ("FORWARD_1", "LKAS_HEARTBIT"),
        ("FORWARD_2", "LKAS_HEARTBIT"),
        ("FORWARD_3", "LKAS_HEARTBIT"),
      ]

      signals += forward_lkas_heartbit_signals
      checks.append(("LKAS_HEARTBIT", 10))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)
