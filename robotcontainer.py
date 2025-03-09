# project imports
import constants
from subsystems.limelight import Limelight
from subsystems.score import Score
from subsystems.climb import Climb
from subsystems.positionalSubsystem import PositionalSubsystem
from subsystems.grabber import Grabber
from subsystems.drivetrain import CommandSwerveDrivetrain
from telemetry import Telemetry

# commands imports
import commands2
import commands2.button
from commands2 import cmd
from commands2.sysid import SysIdRoutine

# wpi imports
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.filter import SlewRateLimiter
from wpimath.units import rotationsToRadians
from phoenix6 import swerve, SignalLogger, utils, hardware
from pathplannerlib.auto import AutoBuilder, NamedCommands

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.

    """

    def __init__(self) -> None:
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        # The robot's subsystems
        self.robotDrive =  CommandSwerveDrivetrain(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            constants.TunerConstants.drivetrain_constants,
            [ constants.TunerConstants.front_left, constants.TunerConstants.front_right, constants.TunerConstants.back_left, constants.TunerConstants.back_right ],
        )

        self.elevator = PositionalSubsystem(
            "elevate",
            motorID=constants.Elevator.mainMotorId,
            followerID=constants.Elevator.othrMotorId,
            motorConfig=constants.Elevator.config,
            conversionRate=constants.Elevator.turnsPerInch,
            sysidConf=constants.Elevator.sysidConf,
            initialTarget=1,
            limitWaitingDistance=1
        )
        self.arm = PositionalSubsystem(
            "arm",
            motorID=constants.Arm.id,
            motorConfig=constants.Arm.config,
            encoderID=constants.Arm.encoder,
            encoderConfig=constants.Arm.encoderConfig,
            sysidConf=constants.Arm.sysidConf,
            limitWaitingDistance=3,
            initialTarget=90
        )
        self.wrist = PositionalSubsystem(
            "wrist",
            motorID=constants.Wrist.id,
            motorConfig=constants.Wrist.config,
            conversionRate=constants.Wrist.gearRatio/360,
            encoderID=constants.Wrist.encoder,
            initialPosition=0.0,
            encoderConfig=constants.Wrist.encoderConfig,
            sysidConf=constants.Wrist.sysidConf,
            limitWaitingDistance=3,
            positionSetErrorBounds=5
        )
        self.grabber = Grabber()

        self.limelight = Limelight(self.robotDrive)
        self.score = Score(
            elevator=self.elevator,
            arm=self.arm,
            wrist=self.wrist,
            grabber=self.grabber
        )
        self.climb = Climb()
        
        # The robot's auton commands
        NamedCommands.registerCommand("Score L1", self.score.l1())
        NamedCommands.registerCommand("Score L2", self.score.l234(constants.scorePositions.l2))
        NamedCommands.registerCommand("Score L3", self.score.l234(constants.scorePositions.l3))
        NamedCommands.registerCommand("Score L4", self.score.l234(constants.scorePositions.l4))

        # The driver's controller
        self.driverController = commands2.button.CommandXboxController(constants.Global.kDriverControllerPort)
        self.operatorController = commands2.button.CommandXboxController(constants.Global.kOperatorControllerPort)
        self.configController = commands2.button.CommandXboxController(constants.Global.kConfigControllerPort)

		# Setting up bindings for necessary control of the swerve drive platform
        self.slewRateX = SlewRateLimiter(2, -3)
        self.slewRateY = SlewRateLimiter(2, -3)
        self.slewRateT = SlewRateLimiter(2, -3)
        self.drive = (
            swerve.requests.FieldCentric()
            .with_deadband(constants.Global.max_speed * 0.1)
            .with_rotational_deadband(
                rotationsToRadians(0.75) * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )

        # Configure the button bindings
        self.configureButtonBindings()

        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """
        alwaysBindAll = False

        if self.driverController.isConnected() or alwaysBindAll:
            print("Binding driver controller")
            # Drive
            self.robotDrive.setDefaultCommand(
                # Drivetrain will execute this command periodically
                self.robotDrive.apply_request(
                    lambda: (
                        self.drive.with_velocity_x(
                            #self.slewRateX.calculate(-self.driverController.getLeftY())
                            -self.driverController.getLeftY()
                            * constants.Global.max_speed
                            * max(not self.robotDrive.slow,constants.Global.break_speed_mul)
                        )  # Drive forward with negative Y (forward)
                        .with_velocity_y(
                            #self.slewRateY.calculate(-self.driverController.getLeftX())
                            -self.driverController.getLeftX()
                            * constants.Global.max_speed
                            * max(not self.robotDrive.slow,constants.Global.break_speed_mul)
                        )  # Drive left with negative X (left)
                        .with_rotational_rate(
                            #self.slewRateT.calculate(self.driverController.getRightX())
                            self.driverController.getRightX()
                            * constants.Global.max_angular_rate
                            * max(not self.robotDrive.slow,constants.Global.break_speed_mul)
                        )  # Drive counterclockwise with X (right)
                    )
                )
            )

            # break on triggers
            (self.driverController.leftTrigger() | self.driverController.rightTrigger()).whileTrue(self.robotDrive.apply_request(lambda: swerve.requests.SwerveDriveBrake()))
            # slow on bumpers
            (self.driverController.leftBumper() | self.driverController.rightBumper()).onTrue(self.robotDrive.slowly(True)).onFalse(self.robotDrive.slowly(False))

            # Run SysId routines when holding back and face buttons.
            # Note that each routine should be run exactly once in a single log.
            self.driverController.povLeft().onTrue(cmd.runOnce(SignalLogger.start))

            (self.driverController.start() & self.driverController.a()).whileTrue(
                self.robotDrive.sys_id_dynamic(SysIdRoutine.Direction.kForward)
            )
            (self.driverController.start() & self.driverController.b()).whileTrue(
                self.robotDrive.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
            )
            (self.driverController.start() & self.driverController.x()).whileTrue(
                self.robotDrive.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
            )
            (self.driverController.start() & self.driverController.y()).whileTrue(
                self.robotDrive.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
            )
            self.driverController.povRight().onTrue(cmd.runOnce(SignalLogger.stop))

            # reset the field-centric heading on start press
            self.driverController.start().onTrue(
                self.limelight.runOnce(lambda: self.limelight.pigeon2.set_yaw(0))
            )

            # modules turn toward their zeros
            self.driverController.back().onTrue(
                self.robotDrive.apply_request(lambda: swerve.requests.PointWheelsAt().with_module_direction(Rotation2d()))
            )

            # go to the closest alignment target
            self.driverController.povUp().whileTrue(self.limelight.pathfind())
            self.driverController.povDown().whileTrue(self.limelight.align())

        if self.operatorController.isConnected() or alwaysBindAll:
            print("Binding operator controller")
            # intake
            (self.operatorController.leftBumper()|self.operatorController.rightBumper()).onTrue(self.score.intake())
            self.operatorController.povLeft().onTrue(cmd.runOnce(self.score.grabber.FWD)).onFalse(cmd.runOnce(self.score.grabber.HLD))
            self.operatorController.povRight().onTrue(cmd.runOnce(self.score.grabber.REV)).onFalse(cmd.runOnce(self.score.grabber.OFF))

            self.operatorController.povUp().onTrue(commands2.SequentialCommandGroup(self.score.position(constants.scorePosition(arm=90,wrist=0)),self.score.position(constants.scorePositions.idle)))

            # score at various heights
            self.operatorController.a().onTrue(self.score.l1())
            self.operatorController.b().onTrue(self.score.l234(constants.scorePositions.l2))
            self.operatorController.x().onTrue(self.score.l234(constants.scorePositions.l3))
            self.operatorController.y().onTrue(self.score.l234(constants.scorePositions.l4))

            # climb
            self.operatorController.leftTrigger().onTrue(commands2.SequentialCommandGroup(self.score.position(constants.scorePosition(arm=155)),self.climb.move(False, constants.Climb.unspoolVoltage)))
            self.operatorController.rightTrigger().onTrue(self.climb.move(True, constants.Climb.climbVoltage))

        if self.configController.isConnected():
            print("Binding config controller")
            # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
            self.configController.leftBumper().onTrue(cmd.runOnce(SignalLogger.start))
            self.configController.a().onTrue(self.score.elevator.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
            self.configController.b().onTrue(self.score.elevator.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
            self.configController.x().onTrue(self.score.elevator.sys_id_dynamic(SysIdRoutine.Direction.kForward))
            self.configController.y().onTrue(self.score.elevator.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
            self.configController.rightBumper().onTrue(cmd.runOnce(SignalLogger.stop))

        if not utils.is_simulation():
            self.logger = Telemetry(constants.Global.max_speed)
            self.robotDrive.register_telemetry(
                lambda state: self.logger.telemeterize(state)
            )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        return self.autoChooser.getSelected()
