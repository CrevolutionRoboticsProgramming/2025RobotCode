package frc.robot.vision;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

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
// import frc.robot.vision.commands.DriveToPoseCommand;

public class LineupMaster {
    private static final Map<ReefFace, Command> leftBranchAlignmentCommands = new HashMap<>();
    // private static final Map<ReefFace, Command> reefCenterAlignmentCommands = new HashMap<>();
    private static final Map<ReefFace, Command> rightBranchAlignmentCommands = new HashMap<>();
    
    public static final PathConstraints pathConstraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));
    
    public static HashSet<ReefFace> avoidReefFacesSet = new HashSet<>(Set.of(ReefFace.BLU_REEF_AB_L4, 
    ReefFace.BLU_REEF_CD_L4, ReefFace.BLU_REEF_EF_L4, ReefFace.BLU_REEF_GH_L4, ReefFace.BLU_REEF_IJ_L4, ReefFace.BLU_REEF_KL_L4,
    ReefFace.RED_REEF_AB_L4, ReefFace.RED_REEF_CD_L4, ReefFace.RED_REEF_EF_L4, ReefFace.RED_REEF_GH_L4, ReefFace.RED_REEF_IJ_L4,
    ReefFace.RED_REEF_KL_L4));

    public static ReefFace[] filteredReefFaces = Arrays.stream(ReefFace.values())
                .filter(e -> !avoidReefFacesSet.contains(e))
                .toArray(ReefFace[]::new);

    
    public LineupMaster() {
        for (ReefFace face : filteredReefFaces) {
            System.out.println("POPULATING WITH ReefFace " + face.name());
            // leftBranchAlignmentCommands.put(face, directDriveToPose(Conversions.rotatePose(face.leftBranch.transformBy(robotOffset), Rotation2d.kZero)));
            // reefCenterAlignmentCommands.put(face, directDriveToPose(Conversions.rotatePose(face.AprilTag.transformBy(robotOffset), Rotation2d.kZero)));
            // rightBranchAlignmentCommands.put(face, directDriveToPose(Conversions.rotatePose(face.rightBranch.transformBy(robotOffset), Rotation2d.kZero)));
            leftBranchAlignmentCommands.put(face, directDriveToPose(() -> face.leftBranch, () -> true));
            rightBranchAlignmentCommands.put(face, directDriveToPose(() -> face.rightBranch, () -> false));
        }
    }

    public static ReefFace getClosestReefFace(Supplier<Pose2d> robotPose){
        double closestDistance = Double.MAX_VALUE; // Distance away from april tag
        ReefFace closestFace = null;

        for (ReefFace face: filteredReefFaces){
            double distance = robotPose.get().getTranslation().getDistance(face.AprilTag.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestFace = face;
            }
        }

        SmartDashboard.putString("closest reef face", closestFace.name());

        return closestFace;
    }

    public Command directDriveToPose(Supplier<Pose2d> targetPose, Supplier<Boolean> isLeftAlign) {
        AutoAlign newAutoAlign = new AutoAlign(targetPose, isLeftAlign);
        return newAutoAlign;
        // return new DriveToPoseCommand(targetPose);
    }

    public Command directDriveToNearestLeftBranch() {
        ReefFace nearestReefFace = getClosestReefFace(() -> PoseEstimatorSubsystem.getInstance().getCurrentPose());
        return directDriveToPose(() -> nearestReefFace.leftBranch, () -> true);
        // return new SelectCommand<>(leftBranchAlignmentCommands, () -> getClosestReefFace(() -> PoseEstimatorSubsystem.getInstance().getCurrentPose()));
        // try {
        //     return new SelectCommand<>(leftBranchAlignmentCommands, () -> getClosestReefFace(PoseEstimatorSubsystem.getInstance().getCurrentPose()));
        // }
        // catch(Exception ex) {
        //     DriverStation.reportError("Direct Drive to nearest left branch failed", ex.getStackTrace());
        //     return null;
        // }
    }

    // public Command directDriveToNearestReefFace() {
    //     return new SelectCommand<>(reefCenterAlignmentCommands, () -> getClosestReefFace(PoseEstimatorSubsystem.getInstance().getCurrentPose()));
    // }

    public Command directDriveToNearestRightBranch() {
        // Supplier<ReefFace> nearestReefFace = getClosestReefFace(()->PoseEstimatorSubsystem.getInstance().getCurrentPose());
        // return directDriveToPose(() -> nearestReefFace.get().rightBranch, false);
        ReefFace nearestReefFace = getClosestReefFace(() -> PoseEstimatorSubsystem.getInstance().getCurrentPose());
        return directDriveToPose(() -> nearestReefFace.leftBranch, () -> true);
        // try {
        // return new SelectCommand<>(rightBranchAlignmentCommands, () -> getClosestReefFace(() -> PoseEstimatorSubsystem.getInstance().getCurrentPose()));
        // } catch(Exception ex) {
        //     DriverStation.reportError("Direct Drive to nearest right branch failed", ex.getStackTrace());
        //     return null;
        // }
    }
    
}
