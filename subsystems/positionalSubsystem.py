# project
from constants import sysidConfig

# standard
from typing import Union, Tuple

# wpilib
import commands2
from wpilib import SmartDashboard
from wpimath import units
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

# vendor
from phoenix6 import configs, controls, SignalLogger
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.hardware.cancoder import CANcoder

class PositionalSubsystem(commands2.Subsystem):
    def __init__(
        self,
        name: str,
        motorID: int,
        motorConfig: configs.TalonFXConfiguration,
        sysidConf: sysidConfig,
        limitWaitingDistance: float,
        positionSetErrorBounds: Union[float, None] = None,
        followerID: Union[int, None] = None,
        conversionRate: units.turns = 1/360,
        # conversionRate is a number such that multiplying it by some external representation will result in the amount of turns necessary to achieve the desired position
        # and deviding some amount of rotations by it will result in the external representation of the position that amount of rotations will achieve
        encoderID: Union[int, None] = None,
        encoderConfig: Union[configs.CANcoderConfiguration, None] = None,
        initialPosition: Union[float, None] = None,
        initialTarget: Union[units.inches, units.degrees] = 0
    ):
        super().__init__()
        self.setName(name)

        self.motor = TalonFX(motorID)
        self.configs = motorConfig
        self.motor.configurator.apply(motorConfig)
        self.conversionRate = conversionRate
        self.limits: Tuple[units.turns] = (-2,2)
        self.limitWaitingDistance: units.turns = limitWaitingDistance * self.conversionRate
        self.positionSetErrorBounds: units.turns = self.conversionRate * positionSetErrorBounds \
            if positionSetErrorBounds is not None else self.limitWaitingDistance

        if followerID is not None:
            self.follower = TalonFX(followerID)
            self.follower.set_control(controls.Follower(motorID, False))
            self.follower.configurator.apply(motorConfig)
        
        if encoderID is not None:
            self.encoder = CANcoder(encoderID)
            if initialPosition is not None:
                self.encoder.set_position(initialPosition)
        
        if initialPosition is not None:
            self.motor.set_position(initialPosition)

        if type(encoderConfig) is float:
            self.encoder.set_position(0)
        
        self.voltage_req = controls.VoltageOut(0)
        self.voltage_req.enable_foc = True

        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage = sysidConf.stepVoltage,
                rampRate = sysidConf.rampRate,
                timeout = sysidConf.timeout,
                recordState = lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state))
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.motor.set_control(self.voltage_req.with_output(volts)),
                lambda log: None,
                self
            )
        )

        self.prevGoal = None
        self.set(initialTarget)

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.dynamic(direction)

    def set(self, target: Union[units.inches,units.degrees,None]):
        if target is not None:
            self.target: units.turns = target*self.conversionRate
    
    def get(self, useTurns: bool = False):
        return self.motor.get_position().value_as_double/(1 if useTurns else self.conversionRate)

    def inPosition(self) -> bool:
        return abs(self.target - self.get(True)) <= self.positionSetErrorBounds

    def limit(self, newLimits: Tuple[units.degrees]):
        self.limits = (newLimits[0]*self.conversionRate,newLimits[1]*self.conversionRate)
        # if newLimits == self.limits:
        #     return
        # self.limits = newLimits
#        self.motor.configurator.apply(
#            self.configs.with_software_limit_switch(
#                configs.SoftwareLimitSwitchConfigs() \
#                    .with_forward_soft_limit_enable(True) \
#                    .with_reverse_soft_limit_enable(True) \
#                    .with_forward_soft_limit_threshold(self.limits[1]) \
#                    .with_reverse_soft_limit_threshold(self.limits[0])
#            )
#        )
        # dont like this
    
    def periodic(self):
        limitTupleIndex = int(self.target > (self.limits[1] + self.limits[0]) / 2)
        goal = self.target if self.limits[1] > self.target > self.limits[0] else \
            self.limits[limitTupleIndex] + self.limitWaitingDistance * (limitTupleIndex * -2 + 1)

        if goal != self.prevGoal:
            self.prevGoal = goal
            self.motor.set_control(controls.MotionMagicVoltage(0).with_position(goal))

        # SmartDashboard.putNumber(self.getName()+"Target",self.target/self.conversionRate)
        # SmartDashboard.putNumber(self.getName()+"LimTarget",goal/self.conversionRate)
        # SmartDashboard.putNumber(self.getName()+"Position",self.get())