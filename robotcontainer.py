# project imports
import constants
from subsystems.limelight import Limelight
from subsystems.elevator import Elevator
from subsystems.pivoter import Pivoter
from subsystems.grabber import Grabber
from subsystems.climb import Climb
from telemetry import Telemetry

# commands imports
import commands2
import commands2.button
from commands2 import cmd
from commands2.sysid import SysIdRoutine

# wpi imports
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from phoenix6 import swerve, SignalLogger, utils
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
        self.robotDrive = constants.TunerConstants.create_drivetrain()
        self.limelight = Limelight(self.robotDrive)
        self.elevator = Elevator()
        self.pivoter = Pivoter()
        self.grabber = Grabber()
        
        # The robot's auton commands
        NamedCommands.registerCommand("Score L1", cmd.none())
        NamedCommands.registerCommand("ground pickup (coral)", cmd.none())
        NamedCommands.registerCommand("Get coral from station", cmd.none())

        # The driver's controller
        self.driverController = commands2.button.CommandXboxController(constants.Global.kDriverControllerPort)
        self.operatorController = commands2.button.CommandXboxController(constants.Global.kOperatorControllerPort)
        self.configController = commands2.button.CommandXboxController(constants.Global.kConfigControllerPort)

		# Setting up bindings for necessary control of the swerve drive platform
        self.drive = (
            swerve.requests.FieldCentric()
            .with_deadband(constants.Global.max_speed * 0.1)
            .with_rotational_deadband(
                constants.Global.max_angular_rate * 0.1
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
        # Drive
        self.robotDrive.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.robotDrive.apply_request(
                lambda: (
                    self.drive.with_velocity_x(
                        -self.driverController.getLeftY()
                        * constants.Global.max_speed
                        * max((self.driverController.leftBumper() | self.driverController.rightBumper()).negate().getAsBoolean(),constants.Global.break_speed_mul)
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self.driverController.getLeftX()
                        * constants.Global.max_speed
                        * max((self.driverController.leftBumper() | self.driverController.rightBumper()).negate().getAsBoolean(),constants.Global.break_speed_mul)
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        self.driverController.getRightX()
                        * constants.Global.max_angular_rate
                    )  # Drive counterclockwise with X (right)
                )
            )
        )

        # break on triggers
        (self.driverController.leftTrigger() | self.driverController.rightTrigger()).whileTrue(self.robotDrive.apply_request(lambda: swerve.requests.SwerveDriveBrake()))

        # Run SysId routines when holding back and face buttons.
        # Note that each routine should be run exactly once in a single log.
        (self.driverController.back() & self.driverController.a()).whileTrue(
            self.robotDrive.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self.driverController.back() & self.driverController.b()).whileTrue(
            self.robotDrive.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self.driverController.back() & self.driverController.x()).whileTrue(
            self.robotDrive.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self.driverController.back() & self.driverController.y()).whileTrue(
            self.robotDrive.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on start press
        self.driverController.start().onTrue(
            self.limelight.runOnce(lambda: self.limelight.pigeon2.set_yaw(0))
        )

        # modules turn toward their zeros
        self.driverController.back().onTrue(
            self.robotDrive.apply_request(lambda: swerve.requests.PointWheelsAt().with_module_direction(Rotation2d()))
		)

        # go to the closest alignment target
        self.driverController.povLeft().onTrue(self.limelight.align())

        (self.operatorController.povDown()|self.configController.povDown()).onTrue(self.elevator.setHeight(0))
        (self.operatorController.povUp()|self.configController.povUp()).onTrue(self.elevator.setHeight(27.3))

        self.operatorController.a().onTrue(self.pivoter.setWristAngle(90))
        self.operatorController.b().onTrue(self.pivoter.setWristAngle(0))

        self.operatorController.x().onTrue(self.pivoter.setArmAngle(90))
        self.operatorController.y().onTrue(self.pivoter.setArmAngle(54.5))

        self.operatorController.leftBumper().onTrue(self.grabber.FWD()).onFalse(self.grabber.OFF())
        self.operatorController.rightBumper().onTrue(self.grabber.REV()).onFalse(self.grabber.OFF())

        # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
        self.configController.leftBumper().onTrue(cmd.runOnce(SignalLogger.start))
        self.configController.rightBumper().onTrue(cmd.runOnce(SignalLogger.stop))
        self.configController.y().whileTrue(self.pivoter.wrist_sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        self.configController.a().whileTrue(self.pivoter.wrist_sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
        self.configController.b().whileTrue(self.pivoter.wrist_sys_id_dynamic(SysIdRoutine.Direction.kForward))
        self.configController.x().whileTrue(self.pivoter.wrist_sys_id_dynamic(SysIdRoutine.Direction.kReverse))

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
