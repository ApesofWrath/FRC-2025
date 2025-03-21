from wpimath.units import degreesToRotations
import commands2

import constants
from subsystems.positionalSubsystem import PositionalSubsystem
from subsystems.grabber import Grabber

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

        self.allowArmRetraction = (-7,160)
        self.disallowArmRetractionFront = (-7,35)
        self.disallowArmRetractionBack = (142,187)
        self.arm.limit(self.allowArmRetraction)

        self.allowElevatorDecent = (.99,55)
        self.disallowElevatorDecent = (26,55)
        self.elevator.limit(self.allowElevatorDecent)

        self.debug = constants.DebugSender("scoreStatus",False)

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
            self.position(constants.scorePosition(arm=20)),
            self.position(constants.scorePosition(wrist=position.wrist)),
            self.position(position),
            self.grabber.intake(),
            self.position(constants.scorePositions.idle),
            commands2.cmd.runOnce(self.grabber.HLD)
        )
        intakeCmd.addRequirements(self)
        return intakeCmd

    def l1(self) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.position(constants.scorePositions.l1),
            self.grabber.outtake(),
            self.position(constants.scorePositions.idle),
            commands2.cmd.runOnce(self.grabber.HLD)
        )

    def l234(self, position: constants.scorePosition) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.debug("elevator up"),
            self.position(constants.scorePosition(elevator = position.elevator)),
            self.debug("rest of pose"),
            self.position(position),
            self.debug("outtake"),
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
            self.debug("returning to position"),
            self.position(constants.scorePosition(arm=constants.scorePositions.idle.arm,wrist=constants.scorePositions.idle.wrist)),
            self.position(constants.scorePositions.idle),
            self.debug("done"),
            commands2.cmd.runOnce(self.grabber.HLD)
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