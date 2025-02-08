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
        motorConfigs.motor_output

        motorConfigs.motor_output = configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)

        motorConfigs.current_limits = configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(80)

        motorConfigs.software_limit_switch = configs.SoftwareLimitSwitchConfigs() \
            .with_forward_soft_limit_enable(True) \
            .with_reverse_soft_limit_enable(True) \
            .with_forward_soft_limit_threshold(30/constants.Elevator.inchPerTurn) \
            .with_reverse_soft_limit_threshold(0)

        # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
        motorConfigs.slot0 = configs.Slot0Configs() \
            .with_k_s(0.082448) \
            .with_k_v(0.11385) \
            .with_k_a(0.00253) \
            .with_k_g(0.22885) \
            .with_k_p(11.629) \
            .with_k_d(0.1818)

        motorConfigs.motion_magic = configs.MotionMagicConfigs() \
            .with_motion_magic_acceleration(200/constants.Elevator.inchPerTurn) \
            .with_motion_magic_cruise_velocity(60/constants.Elevator.inchPerTurn) \
            .with_motion_magic_jerk(2000/constants.Elevator.inchPerTurn)

        self.mainMotor.configurator.apply(motorConfigs)
        self.othrMotor.configurator.apply(motorConfigs)

        self.voltage_req = controls.VoltageOut(0)

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
        self.target = target

    def periodic(self) -> None:
        super().periodic()

        request = controls.MotionMagicVoltage(0)
        self.mainMotor.set_control(
            request.with_position(self.target / constants.Elevator.inchPerTurn)
        )

        SmartDashboard.putNumber("main elev pos", self.mainMotor.get_position().value_as_double * constants.Elevator.inchPerTurn)
        SmartDashboard.putNumber("other elev pos", self.othrMotor.get_position().value_as_double * constants.Elevator.inchPerTurn)
        SmartDashboard.putNumber("elev target", self.target)