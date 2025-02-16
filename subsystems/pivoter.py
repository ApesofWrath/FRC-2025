# standard imports
import constants

# wpi imports
import commands2
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpilib import SmartDashboard

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6 import SignalLogger, controls, configs, signals

class Pivoter(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.voltage_req = controls.VoltageOut(0)
        self.voltage_req.enable_foc = True

        self.armMotor = TalonFX(constants.Pivoter.Arm.id)
        self.armMotor.set_position(0)

        '''armConfigs = configs.TalonFXConfiguration()
        armConfigs.motor_output = configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)
        armConfigs.current_limits = configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(constants.Pivoter.Arm.currentLimit)
        armConfigs.software_limit_switch = configs.SoftwareLimitSwitchConfigs() \
            .with_forward_soft_limit_enable(True) \
            .with_reverse_soft_limit_enable(True) \
            .with_forward_soft_limit_threshold(constants.Pivoter.Arm.forwardLimit) \
            .with_reverse_soft_limit_threshold(constants.Pivoter.Arm.reverseLimit)
        # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
        armConfigs.slot0 = configs.Slot0Configs() \
            .with_k_s(constants.Pivoter.Arm.s) \
            .with_k_v(constants.Pivoter.Arm.v) \
            .with_k_a(constants.Pivoter.Arm.a) \
            .with_k_g(constants.Pivoter.Arm.g) \
            .with_k_p(constants.Pivoter.Arm.p) \
            .with_k_d(constants.Pivoter.Arm.d)
        armConfigs.motion_magic = configs.MotionMagicConfigs() \
            .with_motion_magic_acceleration(constants.Pivoter.Arm.acceleration) \
            .with_motion_magic_cruise_velocity(constants.Pivoter.Arm.velocity) \
            .with_motion_magic_jerk(constants.Pivoter.Arm.jerk)
        self.armMotor.configurator.apply(armConfigs)'''

        self.arm_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage = constants.Pivoter.Arm.stepVoltage,
                recordState = lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state)),
                timeout = constants.Pivoter.Arm.timeout
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.armMotor.set_control(self.voltage_req.with_output(volts)),
                lambda log: None,
                self
            )
        )

        self.wristMotor = TalonFX(constants.Pivoter.Wrist.id)
        self.wristMotor.set_position(0)

        '''wristConfigs = configs.TalonFXConfiguration()
        wristConfigs.motor_output = configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)
        wristConfigs.current_limits = configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(constants.Pivoter.Wrist.currentLimit)
        wristConfigs.software_limit_switch = configs.SoftwareLimitSwitchConfigs() \
            .with_forward_soft_limit_enable(True) \
            .with_reverse_soft_limit_enable(True) \
            .with_forward_soft_limit_threshold(constants.Pivoter.Wrist.forwardLimit) \
            .with_reverse_soft_limit_threshold(constants.Pivoter.Wrist.reverseLimit)
        # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
        wristConfigs.slot0 = configs.Slot0Configs() \
            .with_k_s(constants.Pivoter.Wrist.s) \
            .with_k_v(constants.Pivoter.Wrist.v) \
            .with_k_a(constants.Pivoter.Wrist.a) \
            .with_k_g(constants.Pivoter.Wrist.g) \
            .with_k_p(constants.Pivoter.Wrist.p) \
            .with_k_d(constants.Pivoter.Wrist.d)
        wristConfigs.motion_magic = configs.MotionMagicConfigs() \
            .with_motion_magic_acceleration(constants.Pivoter.Wrist.acceleration) \
            .with_motion_magic_cruise_velocity(constants.Pivoter.Wrist.velocity) \
            .with_motion_magic_jerk(constants.Pivoter.Wrist.jerk)
        self.wristMotor.configurator.apply(wristConfigs)'''

        self.wrist_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage = constants.Pivoter.Wrist.stepVoltage,
                recordState = lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state)),
                timeout = None
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.wristMotor.set_control(self.voltage_req.with_output(volts)),
                lambda log: constants.Pivoter.Wrist.timeout,
                self
            )
        )

    def arm_sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.arm_sys_id_routine.quasistatic(direction)

    def arm_sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.arm_sys_id_routine.dynamic(direction)

    def setState(self, state) -> None:
        self.state = state

    def periodic(self) -> None:
        super().periodic()