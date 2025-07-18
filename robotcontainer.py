## \mainpage
# This is the documentation for [the codebase of Maurice](https://github.com/ApesofWrath/FRC-2025), our robot for the FRC game [Reefscape](https://en.wikipedia.org/wiki/Reefscape).
#
# To navigate this page, I suggest navigating by Classes or Files.

# project imports
import constants
from constants import Direction
from subsystems.vision.vision import Limelight
from subsystems.score import Score
from subsystems.climb import Climb
from subsystems.positionalSubsystem import PositionalSubsystem
from subsystems.grabber import Grabber
from subsystems.drivetrain import CommandSwerveDrivetrain
from telemetry import Telemetry
from subsystems.vision.aligncmd import PIDAlignCMD

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
        SignalLogger.stop()

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
        NamedCommands.registerCommand("Score L1", self.score.l1(constants.scorePositions.l1f))
        NamedCommands.registerCommand("Score L1 no outtake", self.score.position(constants.scorePositions.l1f))
        NamedCommands.registerCommand("Score L2", self.score.l234(constants.scorePositions.l2f))
        NamedCommands.registerCommand("Score L3", self.score.l234(constants.scorePositions.l3f))
        NamedCommands.registerCommand("Score L4", self.score.l234(constants.scorePositions.l4f))
        NamedCommands.registerCommand("Score L4 no outtake", self.score.position(constants.scorePosition(elevator=constants.scorePositions.l4f.elevator)    ))
        NamedCommands.registerCommand("Grabber HLD", cmd.runOnce(self.score.grabber.HLD))
        NamedCommands.registerCommand("Grabber REV", cmd.runOnce(self.score.grabber.REV))
        NamedCommands.registerCommand("Grabber OFF", cmd.runOnce(self.score.grabber.OFF))
        NamedCommands.registerCommand("Grabber FWD", cmd.runOnce(self.score.grabber.FWD))
        NamedCommands.registerCommand("Human player intake position", self.score.position(constants.scorePositions.hpintakeback))
        NamedCommands.registerCommand("Human player intake", self.score.intake(constants.scorePositions.hpintakeback)) # TODO: ???? bad
        NamedCommands.registerCommand("Intake", self.score.intake(constants.scorePositions.intake))
        NamedCommands.registerCommand("Back Intake", self.score.intake(constants.scorePositions.intakeback))
        NamedCommands.registerCommand("Outtake", self.score.grabber.outtake())
        NamedCommands.registerCommand("Target RD", cmd.runOnce(lambda: self.limelight.update_target(constants.Direction.RIGHT, True)))
        NamedCommands.registerCommand("Target LD", cmd.runOnce(lambda: self.limelight.update_target(constants.Direction.LEFT, True)))
        NamedCommands.registerCommand("Target RU", cmd.runOnce(lambda: self.limelight.update_target(constants.Direction.RIGHT, False)))
        NamedCommands.registerCommand("Target LU", cmd.runOnce(lambda: self.limelight.update_target(constants.Direction.LEFT, False)))
        NamedCommands.registerCommand("Align",PIDAlignCMD(self.robotDrive,self.limelight))
        NamedCommands.registerCommand("Slow Align",PIDAlignCMD(self.robotDrive,self.limelight,0.75))
        NamedCommands.registerCommand("Brake", self.robotDrive.apply_request(lambda: swerve.requests.SwerveDriveBrake()))

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
        alwaysBindAll = True

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
                            * constants.Global.max_speed * .25
                            * max(not self.robotDrive.slow,constants.Global.break_speed_mul)
                        )  # Drive forward with negative Y (forward)
                        .with_velocity_y(
                            #self.slewRateY.calculate(-self.driverController.getLeftX())
                            -self.driverController.getLeftX()
                            * constants.Global.max_speed * .25
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

            # break
            (self.driverController.leftTrigger() | self.driverController.rightTrigger()).whileTrue(self.robotDrive.apply_request(lambda: swerve.requests.SwerveDriveBrake()))
            # slow
            (self.driverController.leftBumper() | self.driverController.rightBumper()).onTrue(self.robotDrive.slowly(True)).onFalse(self.robotDrive.slowly(False))

            self.driverController.povLeft().whileTrue(commands2.cmd.runOnce(lambda: self.limelight.adjustGyro(-0.1)))
            self.driverController.povRight().whileTrue(commands2.cmd.runOnce(lambda: self.limelight.adjustGyro(0.1)))
            self.driverController.povUp().whileTrue(commands2.cmd.runOnce(lambda: self.limelight.adjustGyro(-5)))
            self.driverController.povDown().whileTrue(commands2.cmd.runOnce(lambda: self.limelight.adjustGyro(5)))

            # reset the field-centric heading on start press
            self.driverController.start().onTrue(
                self.limelight.runOnce(lambda: self.limelight.pigeon2.set_yaw(0))
            )

            # modules turn toward their zeros
            self.driverController.back().onTrue(
                self.robotDrive.apply_request(lambda: swerve.requests.PointWheelsAt().with_module_direction(Rotation2d()))
            )

            # go to the closest alignment target
            bindalign = lambda trigger, right, high: trigger.whileTrue(commands2.SequentialCommandGroup(
                cmd.runOnce(lambda: self.limelight.update_target(right, high)),
                PIDAlignCMD(self.robotDrive,self.limelight)
            ))
            
            bindalign(self.driverController.a(), Direction.LEFT, False)
            bindalign(self.driverController.b(), Direction.RIGHT, False)
            bindalign(self.driverController.x(), Direction.LEFT, True)
            bindalign(self.driverController.y(), Direction.RIGHT, True)

        if self.operatorController.isConnected() or alwaysBindAll:
            print("Binding operator controller")
            # intake
            self.operatorController.rightBumper().onTrue(self.score.intake(constants.scorePositions.intake))
            self.operatorController.rightTrigger().onTrue(self.score.hpintake(constants.scorePositions.hpintake))
            self.operatorController.leftBumper().onTrue(self.score.intake(constants.scorePositions.intakeback))
            self.operatorController.leftTrigger().onTrue(self.score.hpintake(constants.scorePositions.hpintakeback))
            self.operatorController.povLeft().onTrue(cmd.runOnce(self.score.grabber.FWD)).onFalse(cmd.runOnce(self.score.grabber.HLD))
            self.operatorController.povRight().onTrue(cmd.runOnce(self.score.grabber.REV)).onFalse(cmd.runOnce(self.score.grabber.OFF))

            # score at various heights
            self.operatorController.a().onTrue(commands2.ConditionalCommand(self.score.l1(constants.scorePositions.l1f),self.score.l1(constants.scorePositions.l1b),lambda: self.limelight.__getattribute__("frontForward")))
            self.operatorController.b().onTrue(commands2.ConditionalCommand(self.score.l234(constants.scorePositions.l2f),self.score.l234(constants.scorePositions.l2b),lambda: self.limelight.__getattribute__("frontForward")))
            self.operatorController.x().onTrue(commands2.ConditionalCommand(self.score.l234(constants.scorePositions.l3f),self.score.l234(constants.scorePositions.l3b),lambda: self.limelight.__getattribute__("frontForward")))
            self.operatorController.y().onTrue(commands2.ConditionalCommand(self.score.l234(constants.scorePositions.l4f),self.score.l234(constants.scorePositions.l4b),lambda: self.limelight.__getattribute__("frontForward")))
            self.operatorController.povUp().onTrue(commands2.SequentialCommandGroup(self.score.position(constants.scorePosition(arm=90,wrist=0)),self.score.position(constants.scorePositions.idle),commands2.cmd.runOnce(self.score.grabber.HLD),self.score.resetElevator()))

            # climb
            self.operatorController.leftStick().onTrue(commands2.SequentialCommandGroup(self.score.position(constants.scorePosition(arm=135)),self.climb.unspool(296.77832)))
            self.operatorController.rightStick().onTrue(self.climb.climb())

        if self.configController.isConnected():
            print("Binding config controller")
            # https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
            (self.configController.leftTrigger()|self.configController.rightTrigger()).onTrue(self.score.position(constants.scorePositions.l1))
            #self.configController.leftBumper().onTrue(cmd.runOnce(SignalLogger.start))
            self.configController.a().whileTrue(self.score.wrist.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
            self.configController.b().whileTrue(self.score.wrist.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
            self.configController.x().whileTrue(self.score.wrist.sys_id_dynamic(SysIdRoutine.Direction.kForward))
            self.configController.y().whileTrue(self.score.wrist.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
            #self.configController.rightBumper().onTrue(cmd.runOnce(SignalLogger.stop))

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
