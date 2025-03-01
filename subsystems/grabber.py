# standard imports
import constants

# wpi imports
import commands2
from wpilib import SmartDashboard

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

    def FWD(self) -> None:
        self.mainMotor.set_control(controls.VoltageOut(constants.Grabber.FWDvelocity))

    def REV(self) -> None:
        self.mainMotor.set_control(controls.VoltageOut(-constants.Grabber.REVvelocity))

    def OFF(self) -> None:
        self.mainMotor.set_control(controls.VoltageOut(0))

    def HLD(self) -> None:
        self.mainMotor.set_control(controls.VoltageOut(0.25))

    def intake(self) -> commands2.Command:
        intakeCmd = commands2.RunCommand(
            self.FWD
        ).andThen(
            commands2.WaitCommand(.1)
        ).until(
            lambda: self.mainMotor.get_torque_current().value_as_double >= 40
        ).andThen(
            commands2.cmd.runOnce(self.HLD)
        )
        intakeCmd.addRequirements(self)
        return intakeCmd

    def outtake(self) -> commands2.Command:
            intakeCmd = commands2.RunCommand(
                self.REV
            ).until(
                # TODO: get torque current values for empty
                lambda: False
            ).andThen(
                commands2.cmd.runOnce(self.OFF)
            )
            intakeCmd.addRequirements(self)
            return intakeCmd

    def periodic(self) -> None:
        SmartDashboard.putNumber("grabberVoltage",self.mainMotor.get_torque_current().value_as_double)