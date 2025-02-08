# standard imports
import constants

# wpi imports
import commands2
from wpimath import units
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpilib import SmartDashboard

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6 import SignalLogger, controls, configs, signals

class Grabber(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.mainMotor = TalonFX(constants.Grabber.mainMotorId)

        self.mainMotor.set_position(0)

        motorConfigs = configs.TalonFXConfiguration()

        motorConfigs.motor_output = configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)

        motorConfigs.current_limits = configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(None)

        motorConfigs.software_limit_switch = configs.SoftwareLimitSwitchConfigs() \
            .with_forward_soft_limit_enable(True) \
            .with_reverse_soft_limit_enable(True) \
            .with_forward_soft_limit_threshold(None) \
            .with_reverse_soft_limit_threshold(None)

        # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
        motorConfigs.slot0 = configs.Slot0Configs() \
            .with_k_s(None) \
            .with_k_v(None) \
            .with_k_a(None) \
            .with_k_g(None) \
            .with_k_p(None) \
            .with_k_d(None)

        motorConfigs.motion_magic = configs.MotionMagicConfigs() \
            .with_motion_magic_acceleration(None) \
            .with_motion_magic_cruise_velocity(None) \
            .with_motion_magic_jerk(None)

        self.mainMotor.configurator.apply(motorConfigs)

        self.voltage_req = controls.VoltageOut(0)

        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage = None,
                recordState = lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state)),
                timeout = None
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.mainMotor.set_control(self.voltage_req.with_output(volts)),
                lambda log: None,
                self
            )
        )

        self.states = Enum("States", ["ON"])  # type: ignore
        self.state = None

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.dynamic(direction)

    def setState(self, state) -> None:
        self.state = state

    def on(self) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.setState(self.states.ON))

    def periodic(self) -> None:
        super().periodic()
        match self.state:
            case self.states.ON:
                pass