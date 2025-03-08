from dataclasses import dataclass
from typing import Union
from phoenix6 import CANBus, configs, signals, swerve
from wpimath.units import inchesToMeters, rotationsToRadians, degreesToRadians, degreesToRotations
from wpimath import units
import commands2.cmd as cmd
from wpimath.geometry import Transform2d
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from math import pi

def makeCommand(func):
    def cmdFn(*args, **kwargs):
        return cmd.runOnce(lambda: func(*args, **kwargs))
    return cmdFn

@dataclass
class sysidConfig:
    stepVoltage: float
    timeout: float
    rampRate: float = 1

@dataclass
class scorePosition:
    wrist: Union[units.degrees,None] = None
    arm: Union[units.degrees,None] = None
    elevator: Union[units.inches,None] = None
    reefDistance: Union[units.inches,None] = None

class Limelight:
    kGyroId = 20
    kLimelightHostnames = [ "limelight-foc", "limelight-boc" ]

    kAlignmentTargets = { id: AprilTagFieldLayout().loadField(AprilTagField.kDefaultField).getTagPose(id).toPose2d().transformBy(Transform2d(.5,0,pi)) for id in list(range(6,12))+list(range(17,23)) }

    class precise:
        move_p = 2
        spin_p = 0.75
        xy_tolerance = 0.01
        theta_tolerance = 0.5

class scorePositions:
    idle = scorePosition(
        wrist = 0,
        arm = 90,
        elevator = 1
    )
    intake = scorePosition(
        wrist = -90,
        arm = -6.75,
        elevator = 2
    )
    l1 = scorePosition(
        wrist = -90,
        arm = 25,
        elevator = 11,
        reefDistance = 19.5
    )
    l2 = scorePosition(
        wrist = 0,
        arm = 40,
        elevator = 4,
        reefDistance = 19.5
    )
    l3 = scorePosition(
        wrist = 0,
        arm = 40,
        elevator = 21,
        reefDistance = 19.5
    )
    l4 = scorePosition(
        wrist = 0,
        arm = 25,
        elevator = 54,
        reefDistance = 26.25
    )

class Elevator:
    mainMotorId: int = 13
    othrMotorId: int = 14
    turnsPerInch: units.turns = 1 / ((8/56) * (45/8) * 2)
    config = configs.TalonFXConfiguration()\
        .with_motor_output(configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)
        )\
        .with_current_limits(configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(80)
        )\
        .with_slot0(configs.Slot0Configs() \
            .with_k_s(0.025325) \
            .with_k_v(0.13136) \
            .with_k_a(0.0046553) \
            .with_k_g(0.58851) \
            .with_k_p(7.6082) \
            .with_k_d(0.14139)
        )\
        .with_motion_magic(configs.MotionMagicConfigs() \
            .with_motion_magic_acceleration(200/turnsPerInch) \
            .with_motion_magic_cruise_velocity(60/turnsPerInch) \
            .with_motion_magic_jerk(2000/turnsPerInch)
        )

    sysidConf = sysidConfig(
        stepVoltage = 4.0,
        timeout = 3.5
    )

class Arm:
    id: int = 15
    encoder: int = 18
    config = configs.TalonFXConfiguration()\
        .with_feedback(configs.FeedbackConfigs()\
            .with_feedback_remote_sensor_id(encoder) \
            .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.REMOTE_CANCODER)
        )\
        .with_motor_output(configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE) \
            .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
        )\
        .with_current_limits(configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(60)
        )\
        .with_slot0(configs.Slot0Configs() \
            .with_k_s(0.34284) \
            .with_k_v(5.9659) \
            .with_k_a(0.5081) \
            .with_k_g(0.44017) \
            .with_k_p(122.58) \
            .with_k_d(22.153)
        )\
        .with_motion_magic(configs.MotionMagicConfigs() \
            .with_motion_magic_acceleration(8) \
            .with_motion_magic_cruise_velocity(1.5) \
            .with_motion_magic_jerk(16)
        )
    encoderConfig = configs.CANcoderConfiguration() \
        .with_magnet_sensor(
            configs.MagnetSensorConfigs() \
                .with_magnet_offset(-0.059570) \
                .with_sensor_direction(signals.SensorDirectionValue.CLOCKWISE_POSITIVE) \
                .with_absolute_sensor_discontinuity_point(1)
        )

    sysidConf = sysidConfig(
        stepVoltage = 5,
        rampRate = 1.5,
        timeout = 3
    )

class Wrist:
    gearRatio: float = 40/15
    id: int = 16
    encoder: int = 19

    config = configs.TalonFXConfiguration() \
        .with_motor_output(configs.MotorOutputConfigs() \
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)
        )\
        .with_feedback(configs.FeedbackConfigs() \
            .with_feedback_remote_sensor_id(encoder) \
            .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.REMOTE_CANCODER)
        )\
        .with_current_limits(configs.CurrentLimitsConfigs() \
            .with_stator_current_limit_enable(True) \
            .with_stator_current_limit(40)
        )\
        .with_slot0(configs.Slot0Configs() \
            .with_k_s(0.75) \
            .with_k_v(6.0118) \
            .with_k_a(0.89434) \
            .with_k_g(0) \
            .with_k_p(18.076) \
            .with_k_d(2.7363)
        )\
        .with_motion_magic(configs.MotionMagicConfigs() \
            .with_motion_magic_acceleration(20 * gearRatio) \
            .with_motion_magic_cruise_velocity(.5 * gearRatio) \
            .with_motion_magic_jerk(40 * gearRatio)
        )

    sysidConf = sysidConfig(
        stepVoltage = 8,
        timeout = 5
    )

    encoderConfig = configs.CANcoderConfiguration() \
        .with_magnet_sensor(
            configs.MagnetSensorConfigs() \
                .with_magnet_offset(0.21728515625) \
                .with_sensor_direction(signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE) \
                .with_absolute_sensor_discontinuity_point(0.5)
        )

class Grabber:
    id: int = 17
    FWDvelocity: int = 8
    REVvelocity: int = -5
    HLDvelocity: int = 4
    currentLimit: int = 30

class TunerConstants:
    """
    Generated by the Tuner X Swerve Project Generator
    https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    """

    # Both sets of gains need to be tuned to your individual robot

    # The steer motor uses any SwerveModule.SteerRequestType control request with the
    # output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    _steer_gains = (
        configs.Slot0Configs()
        .with_k_p(100)
        .with_k_i(0)
        .with_k_d(0.5)
        .with_k_s(0.1)
        .with_k_v(2.66)
        .with_k_a(0)
        .with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
    )
    # When using closed-loop control, the drive motor uses the control
    # output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    _drive_gains = (
        configs.Slot0Configs()
        .with_k_p(0.1)
        .with_k_i(0)
        .with_k_d(0)
        .with_k_s(0)
        .with_k_v(0.124)
    )

    # The closed-loop output type to use for the steer motors;
    # This affects the PID/FF gains for the steer motors
    _steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE
    # The closed-loop output type to use for the drive motors;
    # This affects the PID/FF gains for the drive motors
    _drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

    # The type of motor used for the drive motor
    _drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
    # The type of motor used for the drive motor
    _steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

    # The remote sensor feedback type to use for the steer motors;
    # When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    _steer_feedback_type = swerve.SteerFeedbackType.FUSED_CANCODER

    # The stator current at which the wheels start to slip;
    # This needs to be tuned to your individual robot
    _slip_current: units.amperes = 120.0

    # Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    # Some configs will be overwritten; check the `with_*_initial_configs()` API documentation.
    _drive_initial_configs = configs.TalonFXConfiguration()
    _steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        # Swerve azimuth does not require much torque output, so we can set a relatively low
        # stator current limit to help avoid brownouts without impacting performance.
        .with_stator_current_limit(60).with_stator_current_limit_enable(True)
    )
    _encoder_initial_configs = configs.CANcoderConfiguration()
    # Configs for the Pigeon 2; leave this None to skip applying Pigeon 2 configs
    _pigeon_configs: configs.Pigeon2Configuration | None = None

    # CAN bus that the devices are located on;
    # All swerve devices must share the same CAN bus
    canbus = CANBus("Drivetrain", "./logs/example.hoot")

    # Theoretical free speed (m/s) at 12 V applied output;
    # This needs to be tuned to your individual robot
    speed_at_12_volts: units.meters_per_second = 4.73

    # Every 1 rotation of the azimuth results in _couple_ratio drive motor turns;
    # This may need to be tuned to your individual robot
    _couple_ratio = 3.5714285714285716

    _drive_gear_ratio = 6.746031746031747
    _steer_gear_ratio = 21.428571428571427
    _wheel_radius: units.meters = inchesToMeters(2)

    _invert_left_side = False
    _invert_right_side = True

    _pigeon_id = 20

    # These are only used for simulation
    _steer_inertia: units.kilogram_square_meters = 0.01
    _drive_inertia: units.kilogram_square_meters = 0.01
    # Simulated voltage necessary to overcome friction
    _steer_friction_voltage: units.volts = 0.2
    _drive_friction_voltage: units.volts = 0.2

    drivetrain_constants = (
        swerve.SwerveDrivetrainConstants()
        .with_can_bus_name(canbus.name)
        .with_pigeon2_id(_pigeon_id)
        .with_pigeon2_configs(_pigeon_configs)
    )

    _constants_creator: swerve.SwerveModuleConstantsFactory[configs.TalonFXConfiguration, configs.TalonFXConfiguration, configs.CANcoderConfiguration] = (
        swerve.SwerveModuleConstantsFactory()
        .with_drive_motor_gear_ratio(_drive_gear_ratio)
        .with_steer_motor_gear_ratio(_steer_gear_ratio)
        .with_coupling_gear_ratio(_couple_ratio)
        .with_wheel_radius(_wheel_radius)
        .with_steer_motor_gains(_steer_gains)
        .with_drive_motor_gains(_drive_gains)
        .with_steer_motor_closed_loop_output(_steer_closed_loop_output)
        .with_drive_motor_closed_loop_output(_drive_closed_loop_output)
        .with_slip_current(_slip_current)
        .with_speed_at12_volts(speed_at_12_volts)
        .with_drive_motor_type(_drive_motor_type)
        .with_steer_motor_type(_steer_motor_type)
        .with_feedback_source(_steer_feedback_type)
        .with_drive_motor_initial_configs(_drive_initial_configs)
        .with_steer_motor_initial_configs(_steer_initial_configs)
        .with_encoder_initial_configs(_encoder_initial_configs)
        .with_steer_inertia(_steer_inertia)
        .with_drive_inertia(_drive_inertia)
        .with_steer_friction_voltage(_steer_friction_voltage)
        .with_drive_friction_voltage(_drive_friction_voltage)
    )

    # Front Left
    _front_left_drive_motor_id = 3
    _front_left_steer_motor_id = 4
    _front_left_encoder_id = 10
    _front_left_encoder_offset = -0.075439453125
    _front_left_steer_motor_inverted = True
    _front_left_encoder_inverted = False

    _front_left_x_pos: units.meters = inchesToMeters(11.625)
    _front_left_y_pos: units.meters = inchesToMeters(11.625)

    # Front Right
    _front_right_drive_motor_id = 7
    _front_right_steer_motor_id = 8
    _front_right_encoder_id = 12
    _front_right_encoder_offset = 0.356689453125
    _front_right_steer_motor_inverted = True
    _front_right_encoder_inverted = False

    _front_right_x_pos: units.meters = inchesToMeters(11.625)
    _front_right_y_pos: units.meters = inchesToMeters(-11.625)

    # Back Left
    _back_left_drive_motor_id = 1
    _back_left_steer_motor_id = 2
    _back_left_encoder_id = 9
    _back_left_encoder_offset = 0.416259765625
    _back_left_steer_motor_inverted = True
    _back_left_encoder_inverted = False

    _back_left_x_pos: units.meters = inchesToMeters(-11.625)
    _back_left_y_pos: units.meters = inchesToMeters(11.625)

    # Back Right
    _back_right_drive_motor_id = 5
    _back_right_steer_motor_id = 6
    _back_right_encoder_id = 11
    _back_right_encoder_offset = -0.228515625
    _back_right_steer_motor_inverted = True
    _back_right_encoder_inverted = False

    _back_right_x_pos: units.meters = inchesToMeters(-11.625)
    _back_right_y_pos: units.meters = inchesToMeters(-11.625)


    front_left = _constants_creator.create_module_constants(
        _front_left_steer_motor_id,
        _front_left_drive_motor_id,
        _front_left_encoder_id,
        _front_left_encoder_offset,
        _front_left_x_pos,
        _front_left_y_pos,
        _invert_left_side,
        _front_left_steer_motor_inverted,
        _front_left_encoder_inverted,
    )
    front_right = _constants_creator.create_module_constants(
        _front_right_steer_motor_id,
        _front_right_drive_motor_id,
        _front_right_encoder_id,
        _front_right_encoder_offset,
        _front_right_x_pos,
        _front_right_y_pos,
        _invert_right_side,
        _front_right_steer_motor_inverted,
        _front_right_encoder_inverted,
    )
    back_left = _constants_creator.create_module_constants(
        _back_left_steer_motor_id,
        _back_left_drive_motor_id,
        _back_left_encoder_id,
        _back_left_encoder_offset,
        _back_left_x_pos,
        _back_left_y_pos,
        _invert_left_side,
        _back_left_steer_motor_inverted,
        _back_left_encoder_inverted,
    )
    back_right = _constants_creator.create_module_constants(
        _back_right_steer_motor_id,
        _back_right_drive_motor_id,
        _back_right_encoder_id,
        _back_right_encoder_offset,
        _back_right_x_pos,
        _back_right_y_pos,
        _invert_right_side,
        _back_right_steer_motor_inverted,
        _back_right_encoder_inverted,
    )

class Global:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1
    kConfigControllerPort = 2
    max_speed = TunerConstants.speed_at_12_volts # desired top speed
    max_angular_rate = rotationsToRadians(0.25)
    break_speed_mul = 0.1
