# standard imports
from constants import Climb as const

# wpi imports
import commands2
from wpilib import Servo
from wpilib import SmartDashboard

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6 import controls, configs, signals

class Climb(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.motor = TalonFX(const.id)
        self.motor.set_position(0)

        self.motor.configurator.apply(
            configs.TalonFXConfiguration()\
            .with_motor_output(
                configs.MotorOutputConfigs() \
                    .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            ) \
            .with_current_limits(
                configs.CurrentLimitsConfigs() \
                    .with_stator_current_limit_enable(True) \
                    .with_stator_current_limit(const.currentLimit)
            )
        )
        self.motor.set_position(0)

        self.servo = Servo(const.servoChannel)
        self.servo.set(1)

        self.voltage_req = controls.VoltageOut(0)


        SmartDashboard.putString("climbstatus","")
        # start in and engadged(?)
        # unengadge servo and go out when the operator presses a button
        # engadge servo and go back in when the operator presses a diferent button

    def move(self, retracting: bool, voltage: float):
        if not retracting:
            return commands2.SequentialCommandGroup(
                commands2.cmd.runOnce(lambda: SmartDashboard.putString("climbstatus","extend start")),
                commands2.cmd.runOnce(lambda: self.servo.set(retracting)),
                commands2.cmd.runOnce(lambda: SmartDashboard.putString("climbstatus","extend servo set")),
                commands2.cmd.runOnce(lambda: self.motor.set_control(controls.VoltageOut(voltage))),
                commands2.cmd.runOnce(lambda: SmartDashboard.putString("climbstatus","extend voltage set")),
                commands2.WaitUntilCommand(
                    lambda:
                        self.motor.get_position().value_as_double > const.unspoolTarget
                    
                ),
                commands2.cmd.runOnce(lambda: SmartDashboard.putString("climbstatus","extend waited")),
                commands2.cmd.runOnce(lambda: self.motor.set_control(controls.VoltageOut(0))),
                commands2.cmd.runOnce(lambda: SmartDashboard.putString("climbstatus","extend voltage zeroed"))

            )
        else:
            return commands2.SequentialCommandGroup(
                commands2.cmd.runOnce(lambda: SmartDashboard.putString("climbstatus","climb start")),
                commands2.cmd.runOnce(lambda: self.servo.set(retracting)),
                commands2.cmd.runOnce(lambda: SmartDashboard.putString("climbstatus","climb servo set")),
                commands2.cmd.runOnce(lambda: self.motor.set_control(controls.VoltageOut(voltage))),
                commands2.cmd.runOnce(lambda: SmartDashboard.putString("climbstatus","climb voltage set")),
                commands2.WaitUntilCommand(
                    lambda:
                        self.motor.get_position().value_as_double < const.climbTarget
                        #or (
                        #    abs(self.motor.get_velocity().value_as_double) < 0.1 \
                        #    and abs(self.motor.get_torque_current().value_as_double) > 29
                        #)
                ),
                commands2.cmd.runOnce(lambda: SmartDashboard.putString("climbstatus","climb waited")),
                commands2.cmd.runOnce(lambda: self.motor.set_control(controls.VoltageOut(0))),
                commands2.cmd.runOnce(lambda: SmartDashboard.putString("climbstatus","climb voltage zeroed"))
            )

    def periodic(self):
        SmartDashboard.putNumber("climbpos",self.motor.get_position().value_as_double)
        SmartDashboard.putNumber("climbvel",self.motor.get_velocity().value_as_double)
        SmartDashboard.putNumber("climbtorque",self.motor.get_torque_current().value_as_double)