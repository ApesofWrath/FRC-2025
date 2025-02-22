from dataclasses import dataclass
from phoenix6 import configs
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs import TalonFXConfiguration

from wpilib import SmartDashboard
from wpilib.event import EventLoop, BooleanEvent
from wpimath.units import degreesToRotations, degrees
import commands2

import constants
from subsystems.elevator import Elevator
from subsystems.grabber import Grabber
from subsystems.pivoter import Pivoter

@dataclass
class DynamicLimitSetting:
    motor: TalonFX
    baseConfig: TalonFXConfiguration
    forward: degrees
    backward: degrees

class Score(commands2.Subsystem):
    def __init__(self):
        self.elevator = Elevator()
        self.grabber = Grabber()
        self.pivoter = Pivoter()

        self.events = EventLoop()

        # TODO: maybe move all conditions to single-condition events and then compose those events instead of having discrete events with composed conditions
        isArmExtendedFront = lambda: self.pivoter.armEncoder.get_position().value_as_double < degreesToRotations(41)
        isArmExtendedBack = lambda: self.pivoter.armEncoder.get_position().value_as_double > degreesToRotations(139)
        isElevatorUp = lambda: self.elevator.mainMotor.get_position().value_as_double > 26/constants.Elevator.inchPerTurn
        isGrabberAngled = lambda: abs(self.pivoter.wristEncoder.get_position().value_as_double) > degreesToRotations(2.5)

        # TODO: can't go directly from trough scoring to intaking on the other side
        allowGrabberRotation = DynamicLimitSetting(
            self.pivoter.wristMotor,
            self.pivoter.wristConfigs,
            91 * constants.Pivoter.Wrist.gearRatio,
            -91 * constants.Pivoter.Wrist.gearRatio
        )
        disallowGrabberRotation = DynamicLimitSetting(
            self.pivoter.wristMotor,
            self.pivoter.wristConfigs,
            2 * constants.Pivoter.Wrist.gearRatio,
            -2 * constants.Pivoter.Wrist.gearRatio
        )

        allowArmRetraction = DynamicLimitSetting(
            self.pivoter.armMotor,
            self.pivoter.armConfigs,
            187,
            -7
        )
        disallowArmRetractionFront = DynamicLimitSetting(
            self.pivoter.armMotor,
            self.pivoter.armConfigs,
            38,
            -7
        )
        disallowArmRetractionBack = DynamicLimitSetting(
            self.pivoter.armMotor,
            self.pivoter.armConfigs,
            187,
            142
        )

        allowElevatorDecent = DynamicLimitSetting(
            self.elevator.mainMotor,
            self.elevator.motorConfigs,
            56/constants.Elevator.inchPerTurn*360,
            3/constants.Elevator.inchPerTurn*360
        )
        disallowElevatorDecent = DynamicLimitSetting(
            self.elevator.mainMotor,
            self.elevator.motorConfigs,
            56/constants.Elevator.inchPerTurn*360,
            26/constants.Elevator.inchPerTurn*360
        )

        armOutsideElevatorZone = BooleanEvent(
            self.events,
            lambda: isArmExtendedFront() or isArmExtendedBack() or isElevatorUp()
        ).debounce(0.2)
        armOutsideElevatorZone.rising().ifHigh(lambda: self.dynamicLimitChange(allowGrabberRotation))
        armOutsideElevatorZone.falling().ifHigh(lambda: self.dynamicLimitChange(disallowGrabberRotation))

        grabberAtAngleInFront = BooleanEvent(
            self.events,
            lambda: isGrabberAngled() and isArmExtendedFront()
        ).debounce(0.2)
        grabberAtAngleInFront.rising().ifHigh(lambda: self.dynamicLimitChange(disallowArmRetractionFront))
        grabberAtAngleInFront.falling().ifHigh(lambda: self.dynamicLimitChange(allowArmRetraction))

        grabberAtAngleInBack = BooleanEvent(
            self.events,
            lambda: isGrabberAngled() and isArmExtendedBack()
        ).debounce(0.2)
        grabberAtAngleInBack.rising().ifHigh(lambda: self.dynamicLimitChange(disallowArmRetractionBack))
        grabberAtAngleInBack.falling().ifHigh(lambda: self.dynamicLimitChange(allowArmRetraction))

        grabberAtAngleAndRisen = BooleanEvent(
            # problem state. avoid.
            self.events,
            lambda: isGrabberAngled() and not (isArmExtendedBack() or isArmExtendedFront())
        ).debounce(0.2)
        grabberAtAngleAndRisen.rising().ifHigh(lambda: self.dynamicLimitChange(disallowElevatorDecent))
        grabberAtAngleAndRisen.falling().ifHigh(lambda: self.dynamicLimitChange(allowElevatorDecent))

    def dynamicLimitChange(self, *args: DynamicLimitSetting):
        # TODO: smarter limits
            # when a motor is limited, also set the goal to the current position
            # stash the actual desired position somewhere
            # when the limit is removed, reapply the actual desired position
        for limset in args:
            limset.motor.configurator.apply(
                limset.baseConfig.with_software_limit_switch(
                    configs.SoftwareLimitSwitchConfigs() \
                        .with_forward_soft_limit_enable(True) \
                        .with_reverse_soft_limit_enable(True) \
                        .with_forward_soft_limit_threshold(degreesToRotations(limset.forward)) \
                        .with_reverse_soft_limit_threshold(degreesToRotations(limset.backward))
                )
            )

    @constants.makeCommand
    def idle(self):
        self.pivoter.setWristAngle(0)
        self.pivoter.setArmAngle(90)
        self.elevator.setHeight(0)

    @constants.makeCommand
    def intake(self):
        self.pivoter.setWristAngle(-90)
        self.pivoter.setArmAngle(-6.75) # TODO: allow negative
        self.elevator.setHeight(5)

    @constants.makeCommand
    def l1(self):
        self.pivoter.setWristAngle(-90)
        self.pivoter.setArmAngle(25)
        self.elevator.setHeight(11)
        # Dist From Base of Reef: 8.5 in

    @constants.makeCommand
    def l2(self):
        self.pivoter.setWristAngle(0)
        self.pivoter.setArmAngle(40)
        self.elevator.setHeight(10)
        # Dist From Base of Reef: 0 in

    @constants.makeCommand
    def l3(self):
        self.pivoter.setWristAngle(0)
        self.pivoter.setArmAngle(40)
        self.elevator.setHeight(26)
        # Dist From Base of Reef: 0 in

    @constants.makeCommand
    def l4(self):
        self.pivoter.setWristAngle(0)
        self.pivoter.setArmAngle(22)
        self.elevator.setHeight(58)
        # Dist From Base of Reef: 5.75in

    def periodic(self) -> None:
        SmartDashboard.putNumber("armEncoder", self.pivoter.armEncoder.get_position().value_as_double*360)
        SmartDashboard.putNumber("wristEncoder", self.pivoter.wristEncoder.get_position().value_as_double*360)
        self.events.poll()