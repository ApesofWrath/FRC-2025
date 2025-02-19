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

class Elevator(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.mainMotor = TalonFX(constants.Elevator.mainMotorId)
        self.othrMotor = TalonFX(constants.Elevator.othrMotorId)
        self.othrMotor.set_control(controls.Follower(constants.Elevator.mainMotorId, False))

        self.mainMotor.set_position(0)
        self.othrMotor.set_position(0)

        motorConfigs = configs.TalonFXConfiguration()

        motorConfigs.motor_output = configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)

        motorConfigs.current_limits = configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(constants.Elevator.currentLimit)

        motorConfigs.software_limit_switch = configs.SoftwareLimitSwitchConfigs() \
            .with_forward_soft_limit_enable(True) \
            .with_reverse_soft_limit_enable(True) \
            .with_forward_soft_limit_threshold(constants.Elevator.forwardLimit) \
            .with_reverse_soft_limit_threshold(constants.Elevator.reverseLimit)

        # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
        motorConfigs.slot0 = configs.Slot0Configs() \
            .with_k_s(constants.Elevator.s) \
            .with_k_v(constants.Elevator.v) \
            .with_k_a(constants.Elevator.a) \
            .with_k_g(constants.Elevator.g) \
            .with_k_p(constants.Elevator.p) \
            .with_k_d(constants.Elevator.d)

        motorConfigs.motion_magic = configs.MotionMagicConfigs() \
            .with_motion_magic_acceleration(constants.Elevator.acceleration) \
            .with_motion_magic_cruise_velocity(constants.Elevator.velocity) \
            .with_motion_magic_jerk(constants.Elevator.jerk)

        self.mainMotor.configurator.apply(motorConfigs)
        self.othrMotor.configurator.apply(motorConfigs)

        self.voltage_req = controls.VoltageOut(0)
        self.voltage_req.enable_foc = True

        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage = 4.0,
                recordState = lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state)),
                timeout = 3.5
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.mainMotor.set_control(self.voltage_req.with_output(volts)),
                lambda log: None,
                self
            )
        )

        self.target: units.inches = 0

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.dynamic(direction)

    @constants.makeCommand
    def setHeight(self, target: units.inches) -> None:
        self.mainMotor.set_control(
            controls.MotionMagicVoltage(0).with_position(target / constants.Elevator.inchPerTurn)
        )