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

        motorConfigs = configs.TalonFXConfiguration()\
            .with_motor_output(
                configs.MotorOutputConfigs() \
                    .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            )\
            .with_current_limits(
                configs.CurrentLimitsConfigs() \
                    .with_stator_current_limit_enable(True) \
                    .with_stator_current_limit(constants.Grabber.currentLimit)
            )\
            .with_slot0(
                configs.Slot0Configs()\
                    .with_k_p(10/16)\
                    .with_k_i(0)\
                    .with_k_d(0)\
                    .with_k_v(10/16)\
                    .with_k_s(0)\
                    .with_k_a(0)\
                    .with_k_g(0)
            )

        self.mainMotor.configurator.apply(motorConfigs)

        self.voltage_req = controls.VoltageOut(0)

        self.debug = constants.DebugSender("grabberVoltage")

    def FWD(self) -> None:
        self.mainMotor.set_control(controls.VelocityVoltage(35))

    def REV(self) -> None:
        self.mainMotor.set_control(controls.VelocityVoltage(-15))

    def OFF(self) -> None:
        self.mainMotor.set_control(controls.VoltageOut(0))

    def ALN(self) -> None:
        self.mainMotor.set_control(controls.TorqueCurrentFOC(80))

    def HLD(self) -> None:
        self.mainMotor.set_control(controls.TorqueCurrentFOC(20))

    def intake(self) -> commands2.Command:
        intakeCmd = commands2.cmd.runOnce(
            self.FWD # TODO: this sucks use a rolling buffer with collections.deque
        ).andThen(
            commands2.cmd.runOnce(lambda: print("went forward"))
        ).andThen(
            commands2.cmd.runOnce(lambda: print("pass thresh 2"))
        ).andThen(
            commands2.WaitUntilCommand(lambda: self.mainMotor.get_torque_current().value_as_double >= 40)
        ).andThen(
            commands2.cmd.runOnce(lambda: print("passed threshhold 3"))
        ).andThen(
            commands2.WaitCommand(.1)
        ).andThen(
            commands2.cmd.runOnce(self.ALN)
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
        self.debug.send(self.mainMotor.get_torque_current().value_as_double)
