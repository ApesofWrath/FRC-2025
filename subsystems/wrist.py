# standard imports
import constants
from enum import Enum

# wpi imports
import commands2
import commands2.cmd as cmd
import wpimath.controller
from wpilib import SmartDashboard

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6 import configs, controls

class Wrist(commands2.PIDSubsystem):
    def __init__(self) -> None:
        super().__init__(
            wpimath.controller.PIDController(
                constants.Spinner.motorPID["p"],
                constants.Spinner.motorPID["i"],
                constants.Spinner.motorPID["d"],
            )
        )

        self.motor = TalonFX(
            constants.Spinner.driveMotorId,
            constants.Global.canivore
        )  # https://api.ctr-electronics.com/phoenix6/release/python/autoapi/phoenix6/hardware/core/core_talon_fx/index.html#phoenix6.hardware.core.core_talon_fx.CoreTalonFX

        motor_cfg = configs.TalonFXConfiguration()
        slot0Config = motor_cfg.slot0
        slot0Config.k_p = constants.Spinner.motorPID["p"]
        slot0Config.k_i = constants.Spinner.motorPID["i"]
        slot0Config.k_d = constants.Spinner.motorPID["d"]
        slot0Config.k_v = constants.Spinner.motorPID["v"]
        self.motor.configurator.apply(motor_cfg)

        self.states = Enum("States", ["ON"])  # type: ignore
        self.state = None

    def setState(self, state) -> None:
        self.state = state

    def on(self) -> commands2.Command:
        return cmd.runOnce(lambda: self.setState(self.states.ON))

    def periodic(self) -> None:
        super().periodic()
        match self.state:
            case self.states.ON:
                pass