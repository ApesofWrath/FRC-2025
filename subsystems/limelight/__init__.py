import concurrent.futures
import math

import commands2
from phoenix6 import utils, swerve
from phoenix6.hardware import Pigeon2
from wpilib import SmartDashboard, Field2d
from wpimath.geometry import Transform2d, Pose2d
from wpimath import units
from pathplannerlib.path import PathConstraints
from pathplannerlib.auto import AutoBuilder
from wpilib import SmartDashboard, Field2d, DriverStation

import constants
from subsystems.limelight.limelight import LimelightHelpers
from subsystems.drivetrain import CommandSwerveDrivetrain as Drivetrain

class Limelight(commands2.Subsystem):
    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drivetrain = drive
        self.pigeon2 = Pigeon2(constants.Limelight.kGyroId, "Drivetrain")
        self.pigeon2.set_yaw((DriverStation.getAlliance() == DriverStation.Alliance.kBlue) * 180)
        self.drivetrain.set_vision_measurement_std_devs((0.7, 0.7, 0.1)) #(0.7, 0.7, 9999999)

        for id,target in constants.Limelight.kAlignmentTargets.items():
            field = Field2d()
            field.setRobotPose(target)
            SmartDashboard.putData("alignTarget " + str(id), field)

        for name in constants.Limelight.kLimelightHostnames:
            LimelightHelpers.set_imu_mode(name,3)

        self.targetOnAField = Field2d()
        self.close = Field2d()

        SmartDashboard.putData("pathTarget",self.targetOnAField)
        self.tpe = concurrent.futures.ThreadPoolExecutor()

    def fetch_limelight_measurements(self, LLHostname: str) -> None:
        """
        Add vision measurement to MegaTag2
        """

        LimelightHelpers.set_robot_orientation(
            LLHostname,
            self.pigeon2.get_yaw().value,
            0,0,0,0,0
        )

        # get botpose estimate with origin on blue side of field
        mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(LLHostname)
        
        # if we are spinning slower than 720 deg/sec and we see tags
        if mega_tag2.tag_count > 0:
            # set and add vision measurement
            return mega_tag2

    def get_current(self) -> Pose2d:
        return self.drivetrain.get_state().pose

    def get_closest_tag(self, current: Pose2d):
        return current.nearest(list(constants.Limelight.kAlignmentTargets.values()))

    def get_target(self, current, right, high) -> Pose2d:
        target = self.get_closest_tag(current)
        offset = Transform2d(
            -units.inchesToMeters(constants.scorePositions.l4.reefDistance if high else constants.scorePositions.l3.reefDistance),
            units.inchesToMeters((1 if right else -1) * 8.5),
            math.pi
        )
        new_target = target.transformBy(offset)
        self.targetOnAField.setRobotPose(new_target)
        SmartDashboard.putData("pathTarget",self.targetOnAField)
        return new_target

    def update_delta(self, right, high):
        current = self.get_current()
        target = self.get_target(current, right, high)
        self.delta = current.log(target)

    def pathfind(self, right: bool, high: bool) -> commands2.Command:
        target = self.get_target(self.get_current(), right, high)
        self.pathcmd = AutoBuilder.pathfindToPose(
            target,
            PathConstraints( 2.5, 2.5, 1, 1 )
        )
        self.pathcmd.schedule()

    def align(self, right, high):
        self.update_delta(right, high)
        self.drivetrain.set_control(
            swerve.requests.FieldCentric() \
                .with_rotational_rate(self.delta.dtheta * constants.Limelight.precise.spin_p * (abs(self.delta.dtheta_degrees) > constants.Limelight.precise.theta_tolerance)) \
                .with_velocity_y(self.delta.dy * constants.Limelight.precise.move_p * (abs(self.delta.dy) > constants.Limelight.precise.xy_tolerance)) \
                .with_velocity_x(self.delta.dx * constants.Limelight.precise.move_p * (abs(self.delta.dx) > constants.Limelight.precise.xy_tolerance))
        )

    def periodic(self) -> None:
        #self.close.setRobotPose(self.get_target(self.get_current(), constants.Direction.LEFT, False))
        #SmartDashboard.putData("target",self.close)
        if self.pigeon2.get_angular_velocity_z_world(False).value > 720:
            return

        futures = [ self.tpe.submit(self.fetch_limelight_measurements, hn) for hn in constants.Limelight.kLimelightHostnames ]
        for future in concurrent.futures.as_completed(futures):
            estimate = future.result()
            if estimate and estimate.tag_count > 0:
                self.drivetrain.add_vision_measurement(estimate.pose, utils.fpga_to_current_time(estimate.timestamp_seconds))