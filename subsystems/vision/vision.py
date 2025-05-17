import concurrent.futures
import math
from typing import List

import commands2
from phoenix6 import utils
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Transform2d, Pose2d
from wpimath import units
from pathplannerlib.path import PathConstraints, PathPlannerPath, Waypoint, IdealStartingState, GoalEndState
from pathplannerlib.auto import AutoBuilder
from wpilib import  Field2d, DriverStation, SmartDashboard

import constants
from subsystems.vision.lib import LimelightHelpers, PoseEstimate
from subsystems.drivetrain import CommandSwerveDrivetrain as Drivetrain

class Limelight(commands2.Subsystem):
    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drivetrain = drive
        self.pigeon2 = Pigeon2(constants.TunerConstants._pigeon_id, "Drivetrain")
        self.drivetrain.set_vision_measurement_std_devs((0.7, 0.7, 0.1)) #(0.7, 0.7, 9999999)

        for target,id in zip(constants.Limelight.kAlignmentTargets,range(len(constants.Limelight.kAlignmentTargets))):
            field = Field2d()
            field.setRobotPose(target)
            SmartDashboard.putData("alignTarget " + str(id), field)

        self.tag_seen = False
        
        self.pathcmd = commands2.Command()
        self.target = Pose2d()
        self.targetOnAField = Field2d()
        self.frontForward = True

        self.posset = False

        SmartDashboard.putData("pathTarget",self.targetOnAField)
        self.tpe = concurrent.futures.ThreadPoolExecutor()

    def fetch_limelight_measurements(self, LLHostname: str):
        """
        Add vision measurement to MegaTag2
        """

        # get botpose estimate with origin on blue side of field
        mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(LLHostname)

        # if we see tags
        if mega_tag2.tag_count > 0:
            # set and add vision measurement
            return mega_tag2
        
    def fetch_limelight_measurements_mt1(self, LLHostname: str):

        # get botpose estimate with origin on blue side of field
        mega_tag1 = LimelightHelpers.get_botpose_estimate_wpiblue(LLHostname)

        # if we see tags
        if mega_tag1.tag_count > 0:
            # set and add vision measurement
            return mega_tag1

    def get_current(self) -> Pose2d:
        return self.drivetrain.get_state().pose

    def get_closest_tag(self, current: Pose2d): # do not inline it on pain of ow
        return current.nearest(constants.Limelight.kAlignmentTargets)

    def update_target(self, side: constants.Direction, high) -> Pose2d:
        current = self.get_current()
        target = self.get_closest_tag(current)

        self.frontForward = abs(current.log(target).dtheta) < abs(current.log(target.transformBy(Transform2d(0,0,math.pi))).dtheta)

        offset = Transform2d(
            -units.inchesToMeters(constants.scorePositions.l4f.reefDistance if high else constants.scorePositions.l3f.reefDistance),
            units.inchesToMeters(side.distance(self.frontForward)),
            math.pi * (not self.frontForward)
        )
        self.target = target.transformBy(offset)
        self.targetOnAField.setRobotPose(self.target)
        SmartDashboard.putData("pathTarget",self.targetOnAField)

    def std_dev_math(self, estimate: PoseEstimate):
        if estimate.tag_count == 0:
            return 0.5, 0.5, 0.5
        avg_dist = sum(f.dist_to_camera for f in estimate.raw_fiducials) / estimate.tag_count
        if avg_dist < 1.5:
            return .0, .0, .05
        return math.inf, math.inf, math.inf

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

        self.pathcmd = AutoBuilder.followPath(path)
        self.pathcmd.addRequirements(self.drivetrain)
        self.pathcmd.schedule()

    def adjustGyro(self, amount: float):
        old_value = self.pigeon2.get_yaw(True).value
        new_value = old_value + amount
        self.pigeon2.set_yaw(new_value)

    def periodic(self) -> None:
        self.tag_seen = False

        # TODO: refactor
        if DriverStation.isEnabled():
            for name in constants.Limelight.kLimelightHostnames:
                LimelightHelpers.set_imu_mode(name,3)
                LimelightHelpers.set_robot_orientation(
                    name,
                    self.pigeon2.get_yaw(True).value,
                    0,0,0,0,0
                )
        elif DriverStation.isDisabled():
            if not self.posset:
                for name in constants.Limelight.kLimelightHostnames:
                    LimelightHelpers.set_imu_mode(name,1)
                    try:
                        llresult = min(filter(lambda r: r is not None, [ self.fetch_limelight_measurements_mt1(ll) for ll in constants.Limelight.kLimelightHostnames ]), key = lambda r: r.avg_tag_dist)
                    except ValueError:
                        llresult = None
                    if llresult is not None:
                        self.pigeon2.set_yaw(llresult.pose.rotation().degrees())
                        self.drivetrain.reset_pose(llresult.pose)
                        LimelightHelpers.set_robot_orientation(
                            name,
                            llresult.pose.rotation().degrees(),
                            0,0,0,0,0
                        )
                # self.posset = True
            else:
                for name in constants.Limelight.kLimelightHostnames:
                    LimelightHelpers.set_imu_mode(name,3)
                try:
                    llresult = min(filter(lambda r: r is not None, [ self.fetch_limelight_measurements(ll) for ll in constants.Limelight.kLimelightHostnames ]), key = lambda r: r.avg_tag_dist)
                except ValueError:
                    llresult = None
                if llresult is not None:
                    self.pigeon2.set_yaw(llresult.pose.rotation().degrees())
                    self.drivetrain.reset_pose(llresult.pose)

        if self.pigeon2.get_angular_velocity_z_world(False).value > 360:
            return

        futures = [ self.tpe.submit(self.fetch_limelight_measurements, hn) for hn in constants.Limelight.kLimelightHostnames ]
        for future in concurrent.futures.as_completed(futures):
            estimate = future.result()
            if estimate and estimate.tag_count > 0:
                #pose = estimate.pose.relativeTo(self.drivetrain.get_state().pose)
                #if DriverStation.isDisabled() or math.sqrt(pose.x ** 2 + pose.y ** 2) <= 1.0:
                self.tag_seen = estimate.raw_fiducials[0].id in list(range(6,12))+list(range(17,23)) or self.tag_seen
                self.drivetrain.add_vision_measurement(
                    estimate.pose,
                    utils.fpga_to_current_time(estimate.timestamp_seconds),
                    self.std_dev_math(estimate)
                )