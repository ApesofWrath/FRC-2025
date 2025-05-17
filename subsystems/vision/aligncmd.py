import math

from commands2 import Command
from commands2.button import Trigger
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from phoenix6.swerve.requests import FieldCentricFacingAngle, ForwardPerspectiveValue, RobotCentric
from wpilib import SmartDashboard

from ..drivetrain import CommandSwerveDrivetrain
from .vision import Limelight
import constants

# thanks 4915
class PIDAlignCMD(Command):
    def __init__(self, swerve: CommandSwerveDrivetrain, vision: Limelight, speedMult: float = 1):
        self.swerve = swerve
        self.vision = vision
        self.endTrigger = Trigger(self.checkDelta).debounce(.1)
        self.speedMult = speedMult
    
    def checkDelta(self):
        diff = self.swerve.get_state().pose.relativeTo(self.goal)
        position = diff.translation().norm() < constants.Limelight.precise.xy_tolerance
        rot = diff.rotation().degrees() < constants.Limelight.precise.theta_tolerance
        sp = self.swerve.get_state().speeds
        speed = math.sqrt(sp.vx**2+sp.vy**2) < .05
        omega = sp.omega < .1
        return position and rot and speed and omega

    def initialize(self):
        self.goal = self.vision.target

    def execute(self):
        goalState = PathPlannerTrajectoryState()
        goalState.pose = self.goal
      
        speedy = self.swerve.autonpid.calculateRobotRelativeSpeeds(self.swerve.get_state().pose,goalState)
        SmartDashboard.putNumber("target_vx", speedy.vx * self.speedMult)
        SmartDashboard.putNumber("target_vy", speedy.vy * self.speedMult)
        SmartDashboard.putNumber("target_omega", speedy.omega * self.speedMult)
        self.swerve.set_control(
            RobotCentric()
                .with_rotational_rate(min(speedy.omega * self.speedMult, math.pi))
                #.with_forward_perspective(ForwardPerspectiveValue.BLUE_ALLIANCE)\
                .with_velocity_x(speedy.vx * self.speedMult)
                .with_velocity_y(speedy.vy * self.speedMult)
                #.with_target_direction(goalState.deltaRot)
        )
    
    def isFinished(self):
        return self.endTrigger.getAsBoolean()