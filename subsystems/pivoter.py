# standard imports
import constants

# wpi imports
import commands2
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpilib.event import EventLoop, BooleanEvent
from wpilib import SmartDashboard
from wpimath.units import degreesToRotations

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.hardware.cancoder import CANcoder
from phoenix6 import SignalLogger, controls, configs, signals

class Pivoter(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.voltage_req = controls.VoltageOut(0)
        self.voltage_req.enable_foc = True

        self.armMotor = TalonFX(constants.Pivoter.Arm.id)

        self.armEncoder = CANcoder(constants.Pivoter.Arm.encoder)
        self.armEncoder.configurator.apply(
            configs.CANcoderConfiguration() \
                .with_magnet_sensor(
                    configs.MagnetSensorConfigs() \
                        .with_magnet_offset(0.679443359375) \
                        .with_sensor_direction(signals.SensorDirectionValue.CLOCKWISE_POSITIVE) \
                        .with_absolute_sensor_discontinuity_point(1)
                )
        )

        self.armConfigs = configs.TalonFXConfiguration()
        self.armConfigs.feedback = configs.FeedbackConfigs() \
            .with_feedback_remote_sensor_id(constants.Pivoter.Arm.encoder) \
            .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.REMOTE_CANCODER)
        self.armConfigs.motor_output = configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE) \
            .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
        self.armConfigs.current_limits = configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(constants.Pivoter.Arm.currentLimit)
        self.armConfigs.software_limit_switch = configs.SoftwareLimitSwitchConfigs() \
            .with_forward_soft_limit_enable(True) \
            .with_reverse_soft_limit_enable(True) \
            .with_forward_soft_limit_threshold(constants.Pivoter.Arm.forwardLimit) \
            .with_reverse_soft_limit_threshold(constants.Pivoter.Arm.reverseLimit)
        # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
        self.armConfigs.slot0 = configs.Slot0Configs() \
            .with_k_s(constants.Pivoter.Arm.s) \
            .with_k_v(constants.Pivoter.Arm.v) \
            .with_k_a(constants.Pivoter.Arm.a) \
            .with_k_g(constants.Pivoter.Arm.g) \
            .with_k_p(constants.Pivoter.Arm.p) \
            .with_k_d(constants.Pivoter.Arm.d) \
            .with_gravity_type(signals.GravityTypeValue.ARM_COSINE)
        self.armConfigs.motion_magic = configs.MotionMagicConfigs() \
            .with_motion_magic_acceleration(constants.Pivoter.Arm.acceleration) \
            .with_motion_magic_cruise_velocity(constants.Pivoter.Arm.velocity) \
            .with_motion_magic_jerk(constants.Pivoter.Arm.jerk)
        self.armMotor.configurator.apply(self.armConfigs)

        self.arm_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage = constants.Pivoter.Arm.stepVoltage,
                rampRate = constants.Pivoter.Arm.rampVoltage,
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

        self.wristEncoder = CANcoder(constants.Pivoter.Wrist.encoder)
        self.wristEncoder.set_position(0)

        self.wristConfigs = configs.TalonFXConfiguration()
        self.wristConfigs.motor_output = configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)
        self.wristConfigs.feedback = configs.FeedbackConfigs() \
            .with_feedback_remote_sensor_id(constants.Pivoter.Wrist.encoder) \
            .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.REMOTE_CANCODER)
        self.wristConfigs.current_limits = configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(constants.Pivoter.Wrist.currentLimit)
        self.wristConfigs.software_limit_switch = configs.SoftwareLimitSwitchConfigs() \
            .with_forward_soft_limit_enable(True) \
            .with_reverse_soft_limit_enable(True) \
            .with_forward_soft_limit_threshold(constants.Pivoter.Wrist.forwardLimit) \
            .with_reverse_soft_limit_threshold(constants.Pivoter.Wrist.reverseLimit)
        # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
        self.wristConfigs.slot0 = configs.Slot0Configs() \
            .with_k_s(constants.Pivoter.Wrist.s) \
            .with_k_v(constants.Pivoter.Wrist.v) \
            .with_k_a(constants.Pivoter.Wrist.a) \
            .with_k_g(constants.Pivoter.Wrist.g) \
            .with_k_p(constants.Pivoter.Wrist.p) \
            .with_k_d(constants.Pivoter.Wrist.d)
        self.wristConfigs.motion_magic = configs.MotionMagicConfigs() \
            .with_motion_magic_acceleration(constants.Pivoter.Wrist.acceleration) \
            .with_motion_magic_cruise_velocity(constants.Pivoter.Wrist.velocity) \
            .with_motion_magic_jerk(constants.Pivoter.Wrist.jerk)
        self.wristMotor.configurator.apply(self.wristConfigs)

        self.wrist_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage = constants.Pivoter.Wrist.stepVoltage,
                recordState = lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state)),
                timeout = constants.Pivoter.Wrist.timeout
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.wristMotor.set_control(self.voltage_req.with_output(volts)),
                lambda log: None,
                self
            )
        )

        self.events = EventLoop()

        armOutsideElevatorZone = BooleanEvent(
            self.events,
            lambda: self.armEncoder.get_position().value_as_double < degreesToRotations(41)
        ).debounce(0.2)
        armOutsideElevatorZone.rising().ifHigh(lambda: print(self.wristMotor.configurator.apply(
            self.wristConfigs.with_software_limit_switch(
                configs.SoftwareLimitSwitchConfigs()\
                    .with_forward_soft_limit_threshold(degreesToRotations(91 * constants.Pivoter.Wrist.gearRatio)) \
                    .with_reverse_soft_limit_threshold(-degreesToRotations(91 * constants.Pivoter.Wrist.gearRatio)) \
                    .with_forward_soft_limit_enable(True) \
                    .with_reverse_soft_limit_enable(True)
            )
        )))
        armOutsideElevatorZone.falling().ifHigh(lambda: print(self.wristMotor.configurator.apply(
            self.wristConfigs.with_software_limit_switch(
                configs.SoftwareLimitSwitchConfigs()\
                    .with_forward_soft_limit_threshold(degreesToRotations(2 * constants.Pivoter.Wrist.gearRatio)) \
                    .with_reverse_soft_limit_threshold(-degreesToRotations(2 * constants.Pivoter.Wrist.gearRatio)) \
                    .with_forward_soft_limit_enable(True) \
                    .with_reverse_soft_limit_enable(True)
            )
        )))

        grabberAtAngle = BooleanEvent(
            self.events,
            lambda: abs(self.wristEncoder.get_position().value_as_double) > degreesToRotations(2.5)
        ).debounce(0.2)
        grabberAtAngle.rising().ifHigh(lambda: self.armMotor.configurator.apply(
            self.armConfigs.with_software_limit_switch(
                configs.SoftwareLimitSwitchConfigs()\
                    .with_forward_soft_limit_threshold(degreesToRotations(41)) \
                    .with_reverse_soft_limit_threshold(degreesToRotations(0)) \
                    .with_forward_soft_limit_enable(True) \
                    .with_reverse_soft_limit_enable(True)
            )
        ))
        grabberAtAngle.falling().ifHigh(lambda: self.armMotor.configurator.apply(
            self.armConfigs.with_software_limit_switch(
                configs.SoftwareLimitSwitchConfigs()\
                    .with_forward_soft_limit_threshold(degreesToRotations(180)) \
                    .with_reverse_soft_limit_threshold(degreesToRotations(0)) \
                    .with_forward_soft_limit_enable(True) \
                    .with_reverse_soft_limit_enable(True)
            )
        ))

        controls.MotionMagicVoltage(0).with_position(degreesToRotations(90))

    def arm_sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.arm_sys_id_routine.quasistatic(direction)

    def arm_sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.arm_sys_id_routine.dynamic(direction)

    def wrist_sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.wrist_sys_id_routine.quasistatic(direction)

    def wrist_sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.wrist_sys_id_routine.dynamic(direction)

    @constants.makeCommand
    def setArmAngle(self, target) -> None:        
        self.armMotor.set_control(
            controls.MotionMagicVoltage(0).with_position(degreesToRotations(target))
        )
    
    @constants.makeCommand
    def setWristAngle(self, target) -> None:        
        self.wristMotor.set_control(
            controls.MotionMagicVoltage(0).with_position(degreesToRotations(target * constants.Pivoter.Wrist.gearRatio))
        )

    def periodic(self) -> None:
        SmartDashboard.putNumber("armEncoder", self.armEncoder.get_position().value_as_double*360)
        SmartDashboard.putNumber("wristEncoder", self.wristEncoder.get_position().value_as_double*360)
        self.events.poll()