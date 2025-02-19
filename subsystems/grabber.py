# standard imports
import constants
from enum import Enum

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

        # motorConfigs.software_limit_switch = configs.SoftwareLimitSwitchConfigs() \
        #     .with_forward_soft_limit_enable(True) \
        #     .with_reverse_soft_limit_enable(True) \
        #     .with_forward_soft_limit_threshold(None) \
        #     .with_reverse_soft_limit_threshold(None)

        # # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
        # motorConfigs.slot0 = configs.Slot0Configs() \
        #     .with_k_s(None) \
        #     .with_k_v(None) \
        #     .with_k_a(None) \
        #     .with_k_g(None) \
        #     .with_k_p(None) \
        #     .with_k_d(None)

        # motorConfigs.motion_magic = configs.MotionMagicConfigs() \
        #     .with_motion_magic_acceleration(None) \
        #     .with_motion_magic_cruise_velocity(None) \
        #     .with_motion_magic_jerk(None)

        self.mainMotor.configurator.apply(motorConfigs)

        self.voltage_req = controls.VoltageOut(0)

        self.states = Enum("States", ["FWD","REV","OFF"])  # type: ignore
        self.state = self.states.OFF

    def setState(self, state) -> None:
        self.state = state

    def FWD(self) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.setState(self.states.FWD))

    def REV(self) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.setState(self.states.REV))

    def OFF(self) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.setState(self.states.OFF))

    def periodic(self) -> None:
        super().periodic()
        match self.state:
            case self.states.FWD:
                request = controls.VoltageOut(constants.Grabber.FWDvelocity)
            case self.states.REV:
                request = controls.VoltageOut(-constants.Grabber.REVvelocity)
            case self.states.OFF:
                request = controls.VoltageOut(0)
        self.mainMotor.set_control(request)