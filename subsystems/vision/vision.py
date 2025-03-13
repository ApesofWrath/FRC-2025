import concurrent.futures
import math
from typing import List

import commands2
from phoenix6 import utils, swerve
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Transform2d, Pose2d, Twist2d, Translation2d, Rotation2d
from wpimath import units
from pathplannerlib.path import PathConstraints, PathPlannerPath, Waypoint, IdealStartingState, GoalEndState
from pathplannerlib.auto import AutoBuilder
from wpilib import SmartDashboard, Field2d, DriverStation

import constants
from subsystems.vision.lib import LimelightHelpers
from subsystems.drivetrain import CommandSwerveDrivetrain as Drivetrain

class Limelight(commands2.Subsystem):
    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drivetrain = drive
        self.pigeon2 = Pigeon2(constants.TunerConstants._pigeon_id, "Drivetrain")
        self.pigeon2.set_yaw(((DriverStation.getAlliance() == DriverStation.Alliance.kBlue) * 180)-90)
        self.drivetrain.reset_pose(Pose2d(0,0,(DriverStation.getAlliance() == DriverStation.Alliance.kBlue) * math.pi))
        self.drivetrain.set_vision_measurement_std_devs((0.7, 0.7, 0.1)) #(0.7, 0.7, 9999999)

        for id,target in constants.Limelight.kAlignmentTargets.items():
            field = Field2d()
            field.setRobotPose(target)
            SmartDashboard.putData("alignTarget " + str(id), field)

        for name in constants.Limelight.kLimelightHostnames:
            LimelightHelpers.set_imu_mode(name,3)
        
        self.pathcmd = commands2.Command()
        self.target = Pose2d()

        self.targetOnAField = Field2d()
        self.close = Field2d()
        self.delta = Twist2d()

        self.imuset = False

        SmartDashboard.putData("pathTarget",self.targetOnAField)
        self.tpe = concurrent.futures.ThreadPoolExecutor()
        SmartDashboard.putBoolean("pathing",False)

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

    def update_target(self, right, high) -> Pose2d:
        target = self.get_closest_tag(self.get_current())
        offset = Transform2d(
            -units.inchesToMeters(constants.scorePositions.l4.reefDistance if high else constants.scorePositions.l3.reefDistance),
            units.inchesToMeters((1 if right else -1) * 11.5),
            0
        )
        new_target = target.transformBy(offset)
        self.targetOnAField.setRobotPose(new_target)
        SmartDashboard.putData("pathTarget",self.targetOnAField)
        self.target = new_target

    def update_delta(self, right, high):
        current = self.get_current()
        target = self.get_target(current, right, high)
        self.delta = target.log(current)

    def pathfind(self, right: bool, high: bool) -> None:
        SmartDashboard.putBoolean("pathing",True)
        target = self.get_target(self.get_current(), right, high)
        return self.getPath(target)

    def align(self, right, high):
        self.update_delta(right, high)
        self.drivetrain.set_control(
            swerve.requests.FieldCentric() \
                .with_rotational_rate(self.delta.dtheta * constants.Limelight.precise.spin_p * (abs(self.delta.dtheta_degrees) > constants.Limelight.precise.theta_tolerance)) \
                .with_velocity_y(self.delta.dy * constants.Limelight.precise.move_p * (abs(self.delta.dy) > constants.Limelight.precise.xy_tolerance)) \
                .with_velocity_x(self.delta.dx * constants.Limelight.precise.move_p * (abs(self.delta.dx) > constants.Limelight.precise.xy_tolerance))
        )

    def std_dev_math(self, estimate):
        if estimate.tag_count == 0:
            return 0.5, 0.5, 0.5

        avg_dist = sum(f.dist_to_camera for f in estimate.raw_fiducials) / estimate.tag_count
        factor = 1 + (avg_dist ** 2 / 30)

        return 0.5 * factor, 0.5 * factor, math.inf if estimate.is_megatag_2 else (0.5 * factor)

    def getPathVelocityHeading(self, speed):
        if abs(speed) < .25:
            diff: Translation2d = (self.target - self.drivetrain.get_state().pose.translation()).translation()
            return self.target.rotation() if diff.norm() < .01 else diff.angle()
        return Rotation2d(speed.vx,speed.vy)

    def getPath(self):
        driveState = self.drivetrain.get_state()
        drivePose = driveState.pose

        waypoints: List[Waypoint] = PathPlannerPath.waypointsFromPoses([
            Pose2d(
                drivePose.translation(),
                drivePose.rotation()
            ),
            self.target
        ])

        if waypoints[0].anchor.distance(waypoints[1].anchor) < .01:
            return
        
        path = PathPlannerPath(
            waypoints,
            PathConstraints( 2, 1.75, math.pi/2, math.pi ),
            IdealStartingState(float(math.sqrt(driveState.speeds.vx**2+driveState.speeds.vy**2)), self.drivetrain.get_state().pose.rotation()),
            GoalEndState(0.,self.target.rotation())
        )

        path.preventFlipping = True

        self.pathcmd = commands2.cmd.runOnce(lambda: SmartDashboard.putBoolean("pathing",True)).andThen(
            AutoBuilder.followPath(path).andThen(
                commands2.cmd.runOnce(lambda: SmartDashboard.putBoolean("pathing",False))))
        self.pathcmd.addRequirements(self.drivetrain)
        self.pathcmd.schedule()

    def periodic(self) -> None:
        #self.close.setRobotPose(self.get_target(self.get_current(), constants.Direction.LEFT, False))
        #SmartDashboard.putData("target",self.close)

        # DO IMU MODE 3 WHEN DISSABLE AND 4 OTHERWISE
        if DriverStation.isEnabled() and not self.imuset:
            self.imuset = True
            for name in constants.Limelight.kLimelightHostnames:
                LimelightHelpers.set_imu_mode(name,4)

        SmartDashboard.putNumberArray("delt",[self.delta.dx,self.delta.dy,self.delta.dtheta_degrees])

        if self.pigeon2.get_angular_velocity_z_world(False).value > 360:
            return

        futures = [ self.tpe.submit(self.fetch_limelight_measurements, hn) for hn in constants.Limelight.kLimelightHostnames ]
        for future in concurrent.futures.as_completed(futures):
            estimate = future.result()
            if estimate and estimate.tag_count > 0:
                self.drivetrain.add_vision_measurement(
                    estimate.pose,
                    utils.fpga_to_current_time(estimate.timestamp_seconds),
                    self.std_dev_math(estimate)
                )