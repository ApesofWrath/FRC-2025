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
from subsystems.vision.lib import LimelightHelpers
from subsystems.drivetrain import CommandSwerveDrivetrain as Drivetrain

class Limelight(commands2.Subsystem):
    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drivetrain = drive
        self.pigeon2 = Pigeon2(constants.TunerConstants._pigeon_id, "Drivetrain")
        self.drivetrain.set_vision_measurement_std_devs((0.7, 0.7, 0.1)) #(0.7, 0.7, 9999999)

        for id,target in constants.Limelight.kAlignmentTargets.items():
            field = Field2d()
            field.setRobotPose(target)
            SmartDashboard.putData("alignTarget " + str(id), field)
        
        self.pathcmd = commands2.Command()
        self.target = Pose2d()
        self.targetOnAField = Field2d()

        self.imuset = False
        self.posset = False

        SmartDashboard.putData("pathTarget",self.targetOnAField)
        self.tpe = concurrent.futures.ThreadPoolExecutor()

    def fetch_limelight_measurements(self, LLHostname: str):
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

        # if we see tags
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
            units.inchesToMeters(constants.Limelight.strafeDistanceRight if right else constants.Limelight.strafeDistanceLeft),
            0
        )
        new_target = target.transformBy(offset)
        self.targetOnAField.setRobotPose(new_target)
        SmartDashboard.putData("pathTarget",self.targetOnAField)
        self.target = new_target

    def std_dev_math(self, estimate):
        if estimate.tag_count == 0:
            return 0.5, 0.5, 0.5

        avg_dist = sum(f.dist_to_camera for f in estimate.raw_fiducials) / estimate.tag_count
        factor = 1 + (avg_dist ** 2 / 30)

        return 0.5 * factor, 0.5 * factor, math.inf if estimate.is_megatag_2 else (0.5 * factor)

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
        # DO IMU MODE 3 WHEN DISSABLE AND 4 OTHERWISE
        if DriverStation.isEnabled() and not self.imuset:
            self.imuset = True
            for name in constants.Limelight.kLimelightHostnames:
                LimelightHelpers.set_imu_mode(name,2)
        elif DriverStation.isDisabled():
            for name in constants.Limelight.kLimelightHostnames:
                LimelightHelpers.set_imu_mode(name,1)
                LimelightHelpers.set_robot_orientation(
                    name,
                    self.pigeon2.get_yaw().value,
                    0,0,0,0,0
                )
            if (DriverStation.isFMSAttached() or DriverStation.isDSAttached()) and not self.posset:
                self.pigeon2.set_yaw(((DriverStation.getAlliance() == DriverStation.Alliance.kBlue) * 180)-90)
                self.drivetrain.reset_pose(Pose2d(0,0,(DriverStation.getAlliance() == DriverStation.Alliance.kBlue) * math.pi-(math.pi/2)))
                self.posset = True

        if self.pigeon2.get_angular_velocity_z_world(False).value > 360:
            return

        self.fetch_limelight_measurements("dsa")

        futures = [ self.tpe.submit(self.fetch_limelight_measurements, hn) for hn in constants.Limelight.kLimelightHostnames ]
        for future in concurrent.futures.as_completed(futures):
            estimate = future.result()
            if estimate and estimate.tag_count > 0:
                pose = estimate.pose.relativeTo(self.drivetrain.get_state().pose)
                if DriverStation.isDisabled() or math.sqrt(pose.x ** 2 + pose.y ** 2) <= 1.0:
                    self.drivetrain.add_vision_measurement(
                        estimate.pose,
                        utils.fpga_to_current_time(estimate.timestamp_seconds),
                        self.std_dev_math(estimate)
                    )