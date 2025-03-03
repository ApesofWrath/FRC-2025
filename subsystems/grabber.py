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
        self.mainMotor.set_control(controls.VoltageOut(constants.Grabber.REVvelocity))

    def OFF(self) -> None:
        self.mainMotor.set_control(controls.VoltageOut(0))

    def HLD(self) -> None:
        self.mainMotor.set_control(controls.VoltageOut(constants.Grabber.HLDvelocity))

    def intake(self) -> commands2.Command:
        intakeCmd = commands2.cmd.runOnce(
            self.FWD # TODO: this sucks use a rolling buffer with collections.deque
        ).andThen(
            commands2.cmd.runOnce(lambda: print("went forward"))
        ).andThen(
            commands2.WaitUntilCommand(lambda: self.mainMotor.get_torque_current().value_as_double >= 30)
        ).andThen(
            commands2.cmd.runOnce(lambda: print("pass thresh 1"))
        ).andThen(
            commands2.WaitUntilCommand(lambda: self.mainMotor.get_torque_current().value_as_double <= 15)
        ).andThen(
            commands2.cmd.runOnce(lambda: print("pass thresh 2"))
        ).andThen(
            commands2.WaitUntilCommand(lambda: self.mainMotor.get_torque_current().value_as_double >= 20)
        ).andThen(
            commands2.cmd.runOnce(lambda: print("passed threshhold 3"))
        ).andThen(
            commands2.WaitCommand(.1)
        ).andThen(
            commands2.cmd.runOnce(self.HLD)
        )
        intakeCmd.addRequirements(self)
        return intakeCmd

    def outtake(self, check = True) -> commands2.Command:
            intakeCmd = commands2.cmd.runOnce(
                self.REV
            ).andThen(
                commands2.cmd.runOnce(lambda: print("going back"))
            ).until(
                lambda: -2 < self.mainMotor.get_torque_current().value_as_double < -10 or (not check)
            ).andThen(
                commands2.cmd.runOnce(lambda: print("passed threshhold"))
            ).andThen(
                commands2.cmd.runOnce(self.OFF)
            )
            intakeCmd.addRequirements(self)
            return intakeCmd

    def periodic(self) -> None:
        SmartDashboard.putNumber("grabberVoltage",self.mainMotor.get_torque_current().value_as_double)