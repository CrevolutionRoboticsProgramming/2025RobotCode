package frc.robot.aprilTagPoses;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElbowPivot.ElbowPivot;

public class CoralGoalPose2d extends SubsystemBase{
    public static CoralGoalPose2d mInstance;

    public static final Map<Integer, Pose2d> leftPoseMap = new HashMap<>();
    public static final Map<Integer, Pose2d> rightPoseMap = new HashMap<>();
    public CoralGoalPose2d() {
        // RED Alliance LEFT Tag Poses 
        leftPoseMap.put(6, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        leftPoseMap.put(7, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        leftPoseMap.put(8, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        leftPoseMap.put(9, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        leftPoseMap.put(10, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        leftPoseMap.put(11, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        // BLUE ALLINCE LEFT Side Goal Pose of each April Tag
        leftPoseMap.put(17, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        leftPoseMap.put(18, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        leftPoseMap.put(19, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        leftPoseMap.put(20, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        leftPoseMap.put(21, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        leftPoseMap.put(22, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );

        /* This is for the Right GOAL Pose of each april tag 
        (this is when facing the april tag) */
        
        // RED Alliance LEFT Tag Poses 
        rightPoseMap.put(6, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        rightPoseMap.put(7, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        rightPoseMap.put(8, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        rightPoseMap.put(9, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        rightPoseMap.put(10, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        rightPoseMap.put(11, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        // BLUE ALLINCE LEFT Side Goal Pose of each April Tag
        rightPoseMap.put(17, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        rightPoseMap.put(18, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        rightPoseMap.put(19, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        rightPoseMap.put(20, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        rightPoseMap.put(21, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
        rightPoseMap.put(22, 
            new Pose2d(
                new Translation2d(
                    Units.inchesToMeters(0), 
                    Units.inchesToMeters(0)
                ), 
                new Rotation2d(0)
            )
        );
    }

    public static CoralGoalPose2d getInstance() {
        if (mInstance == null) {
            mInstance = new CoralGoalPose2d();
        }
        return mInstance;
    }

    public static Pose2d getLeftTagGoalPose(int tagNum) {
        return leftPoseMap.get(tagNum);
    }

    public static Pose2d getRightTagGoalPose(int tagNum) {
        return rightPoseMap.get(tagNum);
    }
}
