package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.crevolib.math.Conversions;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.vision.VisionConfig.ReefFace;
import frc.robot.vision.commands.AutoAlign;
import frc.robot.vision.commands.DriveToPoseCommand;

public class LineupMaster {
    private static final Map<ReefFace, Command> leftBranchAlignmentCommands = new HashMap<>();
    private static final Map<ReefFace, Command> reefCenterAlignmentCommands = new HashMap<>();
    private static final Map<ReefFace, Command> rightBranchAlignmentCommands = new HashMap<>();
    public static final Transform2d robotOffset = new Transform2d(0.3018, 0, Rotation2d.kZero);
    public static final PathConstraints pathConstraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));
    public LineupMaster() {
        for (ReefFace face : ReefFace.values()) {
            leftBranchAlignmentCommands.put(face, directDriveToPose(Conversions.rotatePose(face.leftBranch.transformBy(robotOffset), Rotation2d.kZero)));
            reefCenterAlignmentCommands.put(face, directDriveToPose(Conversions.rotatePose(face.AprilTag.transformBy(robotOffset), Rotation2d.kZero)));
            rightBranchAlignmentCommands.put(face, directDriveToPose(Conversions.rotatePose(face.rightBranch.transformBy(robotOffset), Rotation2d.kZero)));
        }
    }

    public static ReefFace getClosestReefFace(Pose2d robotPose){
        double closestDistance = Double.MAX_VALUE; // Distance away from april tag
        ReefFace closestFace = null;

        for (ReefFace face: ReefFace.values()){
            double distance = robotPose.getTranslation().getDistance(face.AprilTag.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestFace = face;
            }
        }

        SmartDashboard.putString("closest reef face", closestFace.name());

        return closestFace;
    }

    public Command directDriveToPose(Pose2d targetPose) {
        return new AutoAlign(targetPose);
    }

    public Command directDriveToNearestLeftBranch() {
        return new SelectCommand<>(leftBranchAlignmentCommands, () -> getClosestReefFace(PoseEstimatorSubsystem.getInstance().getCurrentPose()));
    }

    public Command directDriveToNearestReefFace() {
        return new SelectCommand<>(reefCenterAlignmentCommands, () -> getClosestReefFace(PoseEstimatorSubsystem.getInstance().getCurrentPose()));
    }

    public Command directDriveToNearestRightBranch() {
        return new SelectCommand<>(rightBranchAlignmentCommands, () -> getClosestReefFace(PoseEstimatorSubsystem.getInstance().getCurrentPose()));
    }
    
}
