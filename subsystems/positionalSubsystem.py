# project
from constants import sysidConfig

# standard
from typing import Union, Tuple

# wpilib
import commands2
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
        motorID: int,
        motorConfig: configs.TalonFXConfiguration,
        sysidConf: sysidConfig,
        limitWaitingDistance: float,
        followerID: Union[int, None] = None,
        conversionRate: float = 1,
        encoderID: Union[int, None] = None,
        encoderConfig: Union[configs.CANcoderConfiguration, float, None] = None,
        initialTarget: Union[units.inches, units.degrees] = 0
    ):
        super().__init__()

        self.motor = TalonFX(motorID)
        self.configs = motorConfig
        self.motor.configurator.apply(motorConfig)
        self.conversionRate = conversionRate
        self.limits = (-2,2)
        self.limitWaitingDistance = limitWaitingDistance

        if followerID is not None:
            self.follower = TalonFX(followerID)
            self.follower.set_control(controls.Follower(motorID, False))
            self.follower.configurator.apply(motorConfig)
        
        if encoderID is not None:
            self.encoder = CANcoder(encoderID)

        if type(encoderConfig) is configs.CANcoderConfiguration:
            self.encoder.configurator.apply(encoderConfig)
        else:
            self.motor.set_position(0)
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

        self.set(initialTarget)

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.dynamic(direction)

    def set(self, target: Union[units.inches,units.degrees]):
        self.target = units.degreesToRotations(target*self.conversionRate)
        

    def limit(self, limits: Tuple[units.degrees]):
        self.limits = limits
        self.motor.configurator.apply(
            self.configs.with_software_limit_switch(
                configs.SoftwareLimitSwitchConfigs() \
                    .with_forward_soft_limit_enable(True) \
                    .with_reverse_soft_limit_enable(True) \
                    .with_forward_soft_limit_threshold(units.degreesToRotations(limits[1])) \
                    .with_reverse_soft_limit_threshold(units.degreesToRotations(limits[0]))
            )
        )
    
    def periodic(self):
        if self.limits[1] > self.target > self.limits[0]:
            self.motor.set_control(controls.MotionMagicVoltage(0).with_position(self.target))
        else:
            limitTupleIndex = int(self.target > (self.limits[1] + self.limits[0]) / 2)
            limitedGoal = self.limits[limitTupleIndex] + self.limitWaitingDistance * -(limitTupleIndex * 2 - 1)
            self.motor.set_control(controls.MotionMagicVoltage(0).with_position(limitedGoal))