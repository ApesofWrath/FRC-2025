from wpilib import SmartDashboard
from wpilib.event import EventLoop, BooleanEvent
from wpimath.units import degreesToRotations
import commands2

import constants
from subsystems.positionalSubsystem import PositionalSubsystem
from subsystems.grabber import Grabber

class Score(commands2.Subsystem):
    def __init__(self):
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
            conversionRate=constants.Wrist.gearRatio,
            encoderID=constants.Wrist.encoder,
            encoderConfig=0.0,
            sysidConf=constants.Wrist.sysidConf,
            limitWaitingDistance=3,
            positionSetErrorBounds=5
        )
        self.grabber = Grabber()

        self.events = EventLoop()

        isArmExtendedFront = lambda: self.arm.encoder.get_position().value_as_double < degreesToRotations(41)  # noqa: E731
        isArmExtendedBack = lambda: self.arm.encoder.get_position().value_as_double > degreesToRotations(139)  # noqa: E731
        isElevatorUp = lambda: self.elevator.motor.get_position().value_as_double > 26/degreesToRotations(constants.Elevator.turnsPerInch)  # noqa: E731
        isGrabberAngled = lambda: abs(self.wrist.encoder.get_position().value_as_double) > degreesToRotations(5)  # noqa: E731

        allowGrabberRotation = (-91, 91)
        disallowGrabberRotation = (-2, 2)
        self.wrist.limit(disallowGrabberRotation)

        allowArmRetraction = (-7,187)
        disallowArmRetractionFront = (-7,38)
        disallowArmRetractionBack = (142,187)
        self.arm.limit(allowArmRetraction)

        allowElevatorDecent = (.99,56)
        disallowElevatorDecent = (26,56)
        self.elevator.limit(allowElevatorDecent)

        armOutsideElevatorZone = BooleanEvent(
            self.events,
            lambda: isArmExtendedFront() or isArmExtendedBack() or isElevatorUp()
        ).debounce(0.2)
        armOutsideElevatorZone.rising().ifHigh(lambda: self.wrist.limit(allowGrabberRotation))
        armOutsideElevatorZone.falling().ifHigh(lambda: self.wrist.limit(disallowGrabberRotation))

        grabberAtAngleInFront = BooleanEvent(
            self.events,
            lambda: isGrabberAngled() and isArmExtendedFront()
        ).debounce(0.2)
        grabberAtAngleInFront.rising().ifHigh(lambda: self.arm.limit(disallowArmRetractionFront))
        grabberAtAngleInFront.falling().ifHigh(lambda: self.arm.limit(allowArmRetraction))

        grabberAtAngleInBack = BooleanEvent(
            self.events,
            lambda: isGrabberAngled() and isArmExtendedBack()
        ).debounce(0.2)
        grabberAtAngleInBack.rising().ifHigh(lambda: self.arm.limit(disallowArmRetractionBack))
        grabberAtAngleInBack.falling().ifHigh(lambda: self.arm.limit(allowArmRetraction))

        grabberAtAngleAndRisen = BooleanEvent(
            # problem state. avoid.
            self.events,
            lambda: isGrabberAngled() and not (isArmExtendedBack() or isArmExtendedFront())
        ).debounce(0.2)
        grabberAtAngleAndRisen.rising().ifHigh(lambda: self.elevator.limit(disallowElevatorDecent))
        grabberAtAngleAndRisen.falling().ifHigh(lambda: self.elevator.limit(allowElevatorDecent))

    def position(self, position: constants.scorePosition) -> commands2.Command:
        positionCommand = commands2.ParallelDeadlineGroup(
            commands2.WaitUntilCommand(lambda: self.arm.inPosition() and self.wrist.inPosition() and self.elevator.inPosition()),
            commands2.RunCommand(lambda: self.wrist.set(position.wrist)),
            commands2.RunCommand(lambda: self.arm.set(position.arm)),
            commands2.RunCommand(lambda: self.elevator.set(position.elevator))
        )
        positionCommand.addRequirements(self)
        return positionCommand

    def intake(self) -> commands2.Command:
        intakeCmd = commands2.SequentialCommandGroup(
            self.position(constants.scorePositions.intake),
            self.grabber.intake()
        )
        intakeCmd.addRequirements(self)
        return intakeCmd
    
    def score(self, levelPosition) -> commands2.Command:
        scoreCmd = commands2.SequentialCommandGroup(
            self.position(levelPosition),
            self.grabber.outtake()
        )
        scoreCmd.addCommands(self)
        return scoreCmd

    def periodic(self) -> None:
        # Put the command schedule on SmartDashboard
        SmartDashboard.putNumber("armEncoder", self.arm.encoder.get_position().value_as_double/self.arm.conversionRate)
        SmartDashboard.putNumber("wristEncoder", self.wrist.encoder.get_position().value_as_double/self.wrist.conversionRate)
        SmartDashboard.putNumber("elevator", self.elevator.motor.get_position().value_as_double/self.elevator.conversionRate)
        self.events.poll()