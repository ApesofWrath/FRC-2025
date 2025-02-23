from wpilib import SmartDashboard
from wpilib.event import EventLoop, BooleanEvent
from wpimath.units import degreesToRotations, degrees
import commands2

import constants
from subsystems.positionalSubsystem import PositionalSubsystem
from subsystems.grabber import Grabber

class Score(commands2.Subsystem):
    def __init__(self):
        self.elevator = PositionalSubsystem(
            motorID=constants.Elevator.mainMotorId,
            followerID=constants.Elevator.othrMotorId,
            motorConfig=constants.Elevator.config,
            conversionRate=constants.Elevator.inchPerDegree,
            sysidConf=constants.Elevator.sysidConf,
            limitWaitingDistance=1/constants.Elevator.inchPerDegree
        )
        self.arm = PositionalSubsystem(
            motorID=constants.Arm.id,
            motorConfig=constants.Arm.config,
            encoderID=constants.Arm.encoder,
            encoderConfig=constants.Arm.encoderConfig,
            sysidConf=constants.Arm.sysidConf,
            limitWaitingDistance=degreesToRotations(3),
            initialTarget=90
        )
        self.wrist = PositionalSubsystem(
            motorID=constants.Wrist.id,
            motorConfig=constants.Wrist.config,
            conversionRate=constants.Wrist.gearRatio,
            encoderID=constants.Wrist.encoder,
            encoderConfig=0.0,
            sysidConf=constants.Wrist.sysidConf,
            limitWaitingDistance=degreesToRotations(3)
        )
        self.grabber = Grabber()

        self.events = EventLoop()

        # TODO: maybe move all conditions to single-condition events and then compose those events instead of having discrete events with composed conditions
        isArmExtendedFront = lambda: self.arm.encoder.get_position().value_as_double < degreesToRotations(41)
        isArmExtendedBack = lambda: self.arm.encoder.get_position().value_as_double > degreesToRotations(139)
        isElevatorUp = lambda: self.elevator.motor.get_position().value_as_double > 26/degreesToRotations(constants.Elevator.inchPerDegree)
        isGrabberAngled = lambda: abs(self.wrist.encoder.get_position().value_as_double) > degreesToRotations(5)

        allowGrabberRotation = (-91 * constants.Wrist.gearRatio, 91 * constants.Wrist.gearRatio)
        disallowGrabberRotation = (-2 * constants.Wrist.gearRatio, 2 * constants.Wrist.gearRatio)
        self.wrist.limit(disallowGrabberRotation)

        allowArmRetraction = (-7,187)
        disallowArmRetractionFront = (-7,38)
        disallowArmRetractionBack = (142,187)
        self.arm.limit(allowArmRetraction)

        allowElevatorDecent = (3/constants.Elevator.inchPerDegree,56/constants.Elevator.inchPerDegree)
        disallowElevatorDecent = (26/constants.Elevator.inchPerDegree,56/constants.Elevator.inchPerDegree)
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

    @constants.makeCommand
    def idle(self):
        self.wrist.set(0)
        self.arm.set(90)
        self.elevator.set(0)

    @constants.makeCommand
    def intake(self):
        self.wrist.set(-90)
        self.arm.set(-6.75)
        self.elevator.set(5)

    @constants.makeCommand
    def l1(self):
        self.wrist.set(-90)
        self.arm.set(25)
        self.elevator.set(11)
        # Dist From Base of Reef: 8.5 in

    @constants.makeCommand
    def l2(self):
        self.wrist.set(0)
        self.arm.set(40)
        self.elevator.set(10)
        # Dist From Base of Reef: 0 in

    @constants.makeCommand
    def l3(self):
        self.wrist.set(0)
        self.arm.set(40)
        self.elevator.set(26)
        # Dist From Base of Reef: 0 in

    @constants.makeCommand
    def l4(self):
        self.wrist.set(0)
        self.arm.set(22)
        self.elevator.set(58)
        # Dist From Base of Reef: 5.75in
    
    @constants.makeCommand
    def testArm(self):
        self.arm.set(22)
    
    @constants.makeCommand
    def testElevator(self):
        self.elevator(10)

    def periodic(self) -> None:
        SmartDashboard.putNumber("armEncoder", self.arm.encoder.get_position().value_as_double*360)
        SmartDashboard.putNumber("wristEncoder", self.wrist.encoder.get_position().value_as_double*360)
        self.events.poll()