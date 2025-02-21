# standard imports
import constants

# wpi imports
import commands2

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6 import controls, configs, signals

class Grabber(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.mainMotor = TalonFX(constants.Grabber.id)

        self.mainMotor.set_position(0)

        motorConfigs = configs.TalonFXConfiguration()

        motorConfigs.motor_output = configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)

        motorConfigs.current_limits = configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(constants.Grabber.currentLimit)

        self.mainMotor.configurator.apply(motorConfigs)

        self.voltage_req = controls.VoltageOut(0)

    @constants.makeCommand
    def FWD(self) -> commands2.Command:
        self.mainMotor.set_control(controls.VoltageOut(constants.Grabber.FWDvelocity))

    @constants.makeCommand
    def REV(self) -> commands2.Command:
        self.mainMotor.set_control(controls.VoltageOut(-constants.Grabber.REVvelocity))

    @constants.makeCommand
    def OFF(self) -> commands2.Command:
        self.mainMotor.set_control(controls.VoltageOut(0))