# project imports
from robot import MyRobot

# wpi imports
import wpilib.simulation as sim
from wpilib import DriverStation, Mechanism2d, SmartDashboard, RobotController, Color8Bit
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

        # ELEVATOR
        self.elevator_gearbox = DCMotor.krakenX60(2)
        self.elevatorMotors = [
            self.robot.score.elevator.motor.sim_state,
            self.robot.score.elevator.follower.sim_state
        ]

        self.sim_elevator_model =  sim.DCMotorSim(
            plant = LinearSystemId.DCMotorSystem( self.elevator_gearbox, .0001, 7),
            gearbox = self.elevator_gearbox,
            measurementStdDevs = [0.01,0]
        )
        self.elevator_physics_model = sim.ElevatorSim(
            plant = LinearSystemId.elevatorSystem(
                motor = self.elevator_gearbox,
                mass = 9.3,
                radius = units.inchesToMeters(1.79/2),
                gearing = 7
            ),
            gearbox = self.elevator_gearbox,
            minHeight = 1,
            maxHeight = units.inchesToMeters(30),
            simulateGravity=True,
            startingHeight=1,
            measurementStdDevs=[0.0, 0.0]
        )

        # ARM
        self.arm_gearbox = DCMotor.krakenX60()
        self.arm_motor = self.robot.score.arm.motor.sim_state

        self.sim_arm_model = sim.DCMotorSim(
            plant = LinearSystemId.DCMotorSystem( self.arm_gearbox, .0001, 1 ),
            gearbox = self.arm_gearbox,
            measurementStdDevs = [0.01,0]
        )
        self.arm_physics_model = sim.SingleJointedArmSim(
            system = LinearSystemId.singleJointedArmSystem(
                motor = self.arm_gearbox,
                J = .0001,
                gearing = 1.0
            ),
            gearbox = self.elevator_gearbox,
            gearing = 1,
            armLength = units.inchesToMeters(24),
            minAngle = units.degreesToRadians(-7),
            maxAngle = units.degreesToRadians(187),
            simulateGravity = True,
            startingAngle = units.degreesToRadians(90)
        )

        # VISUALIZATION
        self.mech2d = Mechanism2d(50, 50)
        self.root = self.mech2d.getRoot("Root", 10, 0)
        self.elevatorMech2d = self.root.appendLigament(
            "Elevator", self.elevator_physics_model.getPositionInches(), 90
        )
        self.armMech2d = self.elevatorMech2d.appendLigament(
            "Arm", 24, self.arm_physics_model.getAngleDegrees(), color = Color8Bit("#992200")
        )
        SmartDashboard.putData("Score", self.mech2d)

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

        # ELEVATOR
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

        # ARM
        self.arm_motor.set_supply_voltage(RobotController.getBatteryVoltage())
        self.arm_motor.set_supply_voltage(RobotController.getBatteryVoltage())
        self.arm_motor.set_raw_rotor_position(units.radiansToRotations(self.sim_arm_model.getAngularPosition()))
        self.arm_motor.set_rotor_velocity(units.radiansToRotations(self.sim_arm_model.getAngularVelocity()))
        self.arm_motor.set_raw_rotor_position(units.radiansToRotations(self.sim_arm_model.getAngularPosition()))
        self.arm_motor.set_rotor_velocity(units.radiansToRotations(self.sim_arm_model.getAngularVelocity()))
        self.sim_arm_model.setInputVoltage(self.arm_motor.motor_voltage)
        self.sim_arm_model.update(tm_diff)
        self.arm_physics_model.setInput(0, self.arm_motor.motor_voltage)
        self.arm_physics_model.update(tm_diff)
        self.armMech2d.setAngle(self.arm_physics_model.getAngleDegrees())