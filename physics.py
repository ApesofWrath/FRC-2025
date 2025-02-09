# project imports
from robot import MyRobot
import constants

# wpi imports
import wpilib.simulation as sim
from wpilib import DriverStation, Mechanism2d, SmartDashboard, RobotController
from wpimath.system.plant import DCMotor, LinearSystemId
from pyfrc.physics.core import PhysicsInterface
from wpimath import units

# vendor imports
from phoenix6 import unmanaged

class PhysicsEngine:
    """
    Simulates a swerve robot
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller
        self.robot = robot.container

        # ELEVATOR INIT
        self.elevator_gearbox = DCMotor.krakenX60(2)
        self.elevatorMotors = [
            self.robot.elevator.mainMotor.sim_state,
            self.robot.elevator.othrMotor.sim_state
        ]

        self.sim_elevator_model =  sim.DCMotorSim(
            plant = LinearSystemId.DCMotorSystem( self.elevator_gearbox, .0001, 7),
            gearbox = self.elevator_gearbox, measurementStdDevs= [0.01,0]
        )
        self.elevator_physics_model = sim.ElevatorSim(
            plant = LinearSystemId.elevatorSystem( motor = self.elevator_gearbox,
                                                   mass = 9.3,
                                                   radius = units.inchesToMeters(1.79/2),
                                                   gearing = 7 ),
            gearbox = self.elevator_gearbox,
            minHeight = 0,
            maxHeight = units.inchesToMeters(30),
            simulateGravity=True,
            startingHeight=0,
            measurementStdDevs=[0.0, 0.0]
        )

        self.mech2d = Mechanism2d(20, 50)
        self.elevatorRoot = self.mech2d.getRoot("Elevator Root", 10, 0)
        self.elevatorMech2d = self.elevatorRoot.appendLigament(
            "Elevator", self.elevator_physics_model.getPositionInches(), 90
        )
        SmartDashboard.putData("Elevator", self.mech2d)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)

        # ELEVATOR SIM
        for m in self.elevatorMotors:
            m.set_supply_voltage(RobotController.getBatteryVoltage())
            m.set_supply_voltage(RobotController.getBatteryVoltage())
            m.set_raw_rotor_position(units.radiansToRotations(self.sim_elevator_model.getAngularPosition()))
            m.set_rotor_velocity(units.radiansToRotations(self.sim_elevator_model.getAngularVelocity()))
            m.set_raw_rotor_position(units.radiansToRotations(self.sim_elevator_model.getAngularPosition()))
            m.set_rotor_velocity(units.radiansToRotations(self.sim_elevator_model.getAngularVelocity()))
        self.sim_elevator_model.setInputVoltage(self.elevatorMotors[0].motor_voltage)
        self.sim_elevator_model.update(tm_diff)
        self.elevator_physics_model.setInput(0, self.elevatorMotors[0].motor_voltage)
        self.elevator_physics_model.setInput(1, self.elevatorMotors[1].motor_voltage)

        self.elevator_physics_model.update(tm_diff)
        self.elevatorMech2d.setLength(self.elevator_physics_model.getPositionInches())