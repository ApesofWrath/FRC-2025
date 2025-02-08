# standard imports
import constants

# wpi imports
import commands2
from wpimath import units
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6 import SignalLogger, controls, configs

class Elevator(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.mainMotor = TalonFX(constants.Elevator.mainMotorId)
        self.othrMotor = TalonFX(constants.Elevator.othrMotorId)
        self.othrMotor.set_control(controls.Follower(constants.Elevator.mainMotorId, False))
        
        limit_configs = configs.CurrentLimitsConfigs()
        limit_configs.stator_current_limit = 120
        limit_configs.stator_current_limit_enable = True
        self.mainMotor.configurator.apply(limit_configs)
        self.othrMotor.configurator.apply(limit_configs)
        # TODO: phoenix software soft limits

        self.voltage_req = controls.VoltageOut(0)

        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage = 4.0,
                recordState = lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state))
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.mainMotor.set_control(self.voltage_req.with_output(volts)),
                lambda log: None,
                self
            )
        )

        self.target: units.inches = 0

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.dynamic(direction)

    @constants.makeCommand
    def setHeight(self, target: units.inches) -> None:
        self.target = target

    def periodic(self) -> None:
        super().periodic()
        self.mainMotor.set_control(
            controls.PositionDutyCycle(self.target / constants.Elevator.inchPerTurn)
        )