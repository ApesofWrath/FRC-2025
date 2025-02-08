# standard imports
import constants
from enum import Enum

# wpi imports
import commands2
import commands2.cmd as cmd
import wpimath.controller
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6 import SignalLogger, controls

class Elevator(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.mainMotor = TalonFX(constants.Elevator.mainMotorId)
        self.othrMotor = TalonFX(constants.Elevator.othrMotorId)
        self.othrMotor.set_control(controls.Follower(constants.Elevator.mainMotorId, False))
        
        self.voltage_req = controls.VoltageOut(0)
        
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 to prevent brownout
                stepVoltage = 4.0,
                # Log state with Phoenix SignalLogger class
                recordState = lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state))
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.mainMotor.set_control(self.voltage_req.with_output(volts)),
                lambda log: None,
                self
            )
        )

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        return self.sys_id_routine.dynamic(direction)

    def periodic(self) -> None:
        super().periodic()