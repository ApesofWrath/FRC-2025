from wpilib.event import EventLoop, BooleanEvent
from wpimath.units import degreesToRotations
from wpilib import SmartDashboard
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
            conversionRate=constants.Wrist.gearRatio/360,
            encoderID=constants.Wrist.encoder,
            initialPosition=0.0,
            encoderConfig=constants.Wrist.encoderConfig,
            sysidConf=constants.Wrist.sysidConf,
            limitWaitingDistance=3,
            positionSetErrorBounds=5
        )
        self.grabber = Grabber()

        self.events = EventLoop()

        allowGrabberRotation = (-91, 91)
        somewhatAllowGrabberRotationFront = (-30, 2)
        somewhatAllowGrabberRotationBack = (-2, 30)
        disallowGrabberRotation = (-2, 2)
        self.wrist.limit(disallowGrabberRotation)

        allowArmRetraction = (-7,95)
        disallowArmRetractionFront = (-7,35)
        disallowArmRetractionBack = (142,187)
        self.arm.limit(allowArmRetraction)

        allowElevatorDecent = (.99,55)
        disallowElevatorDecent = (26,55)
        self.elevator.limit(allowElevatorDecent)

        self.isArmExtendedFront = False
        self.isArmExtendedBack = False
        self.isArmSomewhatExtendedFront = False
        self.isArmSomewhatExtendedBack = False
        self.isElevatorUp = False
        self.isGrabberAngled = False

        armOutsideElevatorZone = BooleanEvent(
            self.events,
            lambda: self.isArmExtendedFront or self.isArmExtendedBack or self.isElevatorUp).debounce(.05)
        armOutsideElevatorZone.rising().ifHigh(lambda: self.wrist.limit(allowGrabberRotation))

        armSomewhatOutsideElevatorZoneFront = BooleanEvent(
            self.events,
            lambda: self.isArmSomewhatExtendedFront
        ).debounce(.05)
        armSomewhatOutsideElevatorZoneFront.rising().ifHigh(lambda: self.wrist.limit(somewhatAllowGrabberRotationFront))

        armSomewhatOutsideElevatorZoneBack = BooleanEvent(
            self.events,
            lambda: self.isArmSomewhatExtendedBack
        ).debounce(.05)
        armSomewhatOutsideElevatorZoneBack.rising().ifHigh(lambda: self.wrist.limit(somewhatAllowGrabberRotationBack))

        armInsideElevatorZone = BooleanEvent(
            self.events,
            lambda: not (self.isArmExtendedBack or self.isArmExtendedFront or self.isArmSomewhatExtendedBack or self.isArmSomewhatExtendedFront or self.isElevatorUp)
        ).debounce(.05)
        armInsideElevatorZone.rising().ifHigh(lambda: self.wrist.limit(disallowGrabberRotation))

        grabberAtAngleInFront = BooleanEvent(
            self.events,
            lambda: self.isGrabberAngled and self.isArmExtendedFront
        ).debounce(.05)
        grabberAtAngleInFront.rising().ifHigh(lambda: self.arm.limit(disallowArmRetractionFront))
        grabberAtAngleInFront.falling().ifHigh(lambda: self.arm.limit(allowArmRetraction))

        grabberAtAngleInBack = BooleanEvent(
            self.events,
            lambda: self.isGrabberAngled and self.isArmExtendedBack
        ).debounce(.05)
        grabberAtAngleInBack.rising().ifHigh(lambda: self.arm.limit(disallowArmRetractionBack))
        grabberAtAngleInBack.falling().ifHigh(lambda: self.arm.limit(allowArmRetraction))

        grabberAtAngleAndRisen = BooleanEvent(
            # problem state. avoid.
            self.events,
            lambda: self.isGrabberAngled and not (self.isArmExtendedBack or self.isArmExtendedFront) and self.isElevatorUp
        ).debounce(0.2)
        grabberAtAngleAndRisen.rising().ifHigh(lambda: self.elevator.limit(disallowElevatorDecent))
        grabberAtAngleAndRisen.falling().ifHigh(lambda: self.elevator.limit(allowElevatorDecent))

        SmartDashboard.putString("scoreStatus","nothing")

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
            self.position(constants.scorePosition(arm=35)),
            self.position(constants.scorePosition(wrist=constants.scorePositions.intake.wrist)),
            self.position(constants.scorePositions.intake),
            self.grabber.intake(),
            self.position(constants.scorePositions.idle)
        )
        intakeCmd.addRequirements(self)
        return intakeCmd

    def l1(self) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.position(constants.scorePositions.l1),
            self.grabber.outtake(),
            self.position(constants.scorePositions.idle)
        )

    def l234(self, position: constants.scorePosition) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            commands2.cmd.runOnce(lambda: SmartDashboard.putString("scoreStatus","elevator up")),
            self.position(constants.scorePosition(elevator = position.elevator)),
            commands2.cmd.runOnce(lambda: SmartDashboard.putString("scoreStatus","rest of pose")),
            self.position(position),
            commands2.cmd.runOnce(lambda: SmartDashboard.putString("scoreStatus","outtake")),
            self.grabber.outtake(False).deadlineWith(
                self.position(
                    constants.scorePosition(
                        wrist = -20,
                        elevator = position.elevator - 3
                    ) if position is not constants.scorePositions.l4 else \
                    constants.scorePosition(
                        elevator = position.elevator - 5
                    )
                ),
            ),
            commands2.cmd.runOnce(lambda: SmartDashboard.putString("scoreStatus","returning to position")),
            self.position(constants.scorePosition(arm=constants.scorePositions.idle.arm,wrist=constants.scorePositions.idle.wrist)),
            self.position(constants.scorePositions.idle),
            commands2.cmd.runOnce(lambda: SmartDashboard.putString("scoreStatus","done")),
        )

    def periodic(self) -> None:
        armStatusValue = self.arm.get()
        self.isArmExtendedFront = armStatusValue <= 30
        self.isArmExtendedBack = armStatusValue >= 150
        self.isArmSomewhatExtendedFront = 70 > armStatusValue > 30
        self.isArmSomewhatExtendedBack = 110 < armStatusValue < 150
        self.isElevatorUp = self.elevator.motor.get_position().value_as_double > 26*constants.Elevator.turnsPerInch
        self.isGrabberAngled = abs(self.wrist.encoder.get_position().value_as_double) > degreesToRotations(5)
        self.events.poll()
