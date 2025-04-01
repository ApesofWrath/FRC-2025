from wpimath.units import degreesToRotations
import commands2

import constants
from subsystems.positionalSubsystem import PositionalSubsystem
from subsystems.grabber import Grabber

from typing import Callable

class Score(commands2.Subsystem):
    def __init__(
            self,
            elevator: PositionalSubsystem,
            arm: PositionalSubsystem,
            wrist: PositionalSubsystem,
            grabber: Grabber
        ):
        self.elevator = elevator
        self.arm = arm 
        self.wrist = wrist 
        self.grabber = grabber 

        self.allowGrabberRotation = (-91, 91)
        self.somewhatAllowGrabberRotationFront = (-30, 2)
        self.somewhatAllowGrabberRotationBack = (-2, 30)
        self.disallowGrabberRotation = (-2, 2)
        self.wrist.limit(self.disallowGrabberRotation)

        self.allowArmRetraction = (-14.25,194.25)
        self.disallowArmRetractionFront = (-14.25,35)
        self.disallowArmRetractionBack = (145,194.25)
        self.arm.limit(self.allowArmRetraction)

        self.allowElevatorDecent = (.99,55)
        self.disallowElevatorDecent = (26,55)
        self.elevator.limit(self.allowElevatorDecent)

        self.debug = constants.DebugSender("scoreStatus",False)
        self.alignDebug = constants.DebugSender("backwards")

    def position(self, position: constants.scorePosition) -> commands2.Command:
        positionCommand = commands2.ParallelDeadlineGroup(
            commands2.WaitUntilCommand(lambda: self.arm.inPosition() and self.wrist.inPosition() and self.elevator.inPosition()),
            commands2.RunCommand(lambda: self.wrist.set(position.wrist)),
            commands2.RunCommand(lambda: self.arm.set(position.arm)),
            commands2.RunCommand(lambda: self.elevator.set(position.elevator))
        )
        positionCommand.addRequirements(self)
        return positionCommand

    def intake(self,position:constants.scorePosition) -> commands2.Command:
        intakeCmd = commands2.SequentialCommandGroup(
            self.position(constants.scorePosition(arm=20 if position.arm < 180 else 160)),
            self.position(constants.scorePosition(wrist=position.wrist)),
            self.position(position),
            self.grabber.intake(),
            self.position(constants.scorePositions.idle),
            commands2.cmd.runOnce(self.grabber.HLD),
            self.resetElevator()
        )
        intakeCmd.addRequirements(self)
        return intakeCmd

    def l1(self, position: constants.scorePosition) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.position(position),
            self.grabber.outtake(),
            self.position(constants.scorePositions.idle),
            commands2.cmd.runOnce(self.grabber.HLD),
            self.resetElevator()
        )

    def l234(self, position: constants.scorePosition) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.debug("elevator up"),
            self.position(constants.scorePosition(elevator = position.elevator)),
            self.debug("rest of pose"),
            self.position(position),
            self.debug("outtake"),
            self.position(
                constants.scorePosition(
                    elevator = position.elevator - (6 if position.elevator == constants.scorePositions.l3f.elevator else
                                                    6 if position.elevator == constants.scorePositions.l4f.elevator else 8)
                )
            ),
            self.grabber.outtake(),
            self.debug("returning to position"),
            self.position(constants.scorePosition(elevator = position.elevator - 14)) if position.elevator == constants.scorePositions.l4f.elevator else commands2.cmd.none(),
            self.position(constants.scorePosition(arm=constants.scorePositions.idle.arm,wrist=constants.scorePositions.idle.wrist)),
            self.position(constants.scorePositions.idle),
            self.debug("done"),
            commands2.cmd.runOnce(self.grabber.HLD),
            self.resetElevator()
        )

    def resetElevator(self) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.position(constants.scorePositions.idle),
            commands2.WaitUntilCommand(self.elevator.inPosition),
            commands2.cmd.runOnce(lambda: self.elevator.setEnabled(False)),
            commands2.WaitUntilCommand(lambda: abs(self.elevator.motor.get_velocity().value_as_double) < .01),
            commands2.cmd.runOnce(lambda: self.elevator.motor.set_position(0)),
            commands2.cmd.runOnce(lambda: self.elevator.setEnabled(True)),
        )

    def periodic(self) -> None:
        armStatusValue = self.arm.get()
        isArmExtendedFront = armStatusValue <= 30
        isArmExtendedBack = armStatusValue >= 150
        isArmSomewhatExtendedFront = 70 > armStatusValue > 30
        isArmSomewhatExtendedBack = 110 < armStatusValue < 150
        isArmSpecialExtension = 60< armStatusValue <120
        isElevatorUp = self.elevator.motor.get_position().value_as_double > 26*constants.Elevator.turnsPerInch
        isGrabberAngled = abs(self.wrist.encoder.get_position().value_as_double) > degreesToRotations(5)

        if isArmExtendedFront or isArmExtendedBack or isElevatorUp:
            self.wrist.limit(self.allowGrabberRotation)
        elif isArmSomewhatExtendedFront:
            self.wrist.limit(self.somewhatAllowGrabberRotationFront)
        elif isArmSomewhatExtendedBack:
            self.wrist.limit(self.somewhatAllowGrabberRotationBack)
        elif isArmSpecialExtension:
            self.wrist.limit(self.disallowGrabberRotation)

        if isGrabberAngled:
            if isArmExtendedFront:
                self.arm.limit(self.disallowArmRetractionFront)
            elif isArmExtendedBack:
                self.arm.limit(self.disallowArmRetractionBack)
        else:
            self.arm.limit(self.allowArmRetraction)

        if isGrabberAngled and not (isArmExtendedBack or isArmExtendedFront) and isElevatorUp:
            self.elevator.limit(self.disallowElevatorDecent)
        else:
            self.elevator.limit(self.allowElevatorDecent)