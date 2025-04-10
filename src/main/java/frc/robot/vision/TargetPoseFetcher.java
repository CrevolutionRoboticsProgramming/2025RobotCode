package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.driver.DriverXbox;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.rushinator.RushinatorWrist;
import frc.robot.vision.VisionConfig.ReefFace;

public class TargetPoseFetcher {

    Map<ReefFace, ReefFace> normalToL4PosMap = new HashMap<>();

    public TargetPoseFetcher() {
        normalToL4PosMap.put(ReefFace.RED_REEF_AB, ReefFace.RED_REEF_AB_L4);
        normalToL4PosMap.put(ReefFace.RED_REEF_CD, ReefFace.RED_REEF_CD_L4);
        normalToL4PosMap.put(ReefFace.RED_REEF_EF, ReefFace.RED_REEF_EF_L4);
        normalToL4PosMap.put(ReefFace.RED_REEF_GH, ReefFace.RED_REEF_GH_L4);
        normalToL4PosMap.put(ReefFace.RED_REEF_IJ, ReefFace.RED_REEF_IJ_L4);
        normalToL4PosMap.put(ReefFace.RED_REEF_KL, ReefFace.RED_REEF_KL_L4);
        
        normalToL4PosMap.put(ReefFace.BLU_REEF_AB, ReefFace.BLU_REEF_AB_L4);
        normalToL4PosMap.put(ReefFace.BLU_REEF_CD, ReefFace.BLU_REEF_CD_L4);
        normalToL4PosMap.put(ReefFace.BLU_REEF_EF, ReefFace.BLU_REEF_EF_L4);
        normalToL4PosMap.put(ReefFace.BLU_REEF_GH, ReefFace.BLU_REEF_GH_L4);
        normalToL4PosMap.put(ReefFace.BLU_REEF_IJ, ReefFace.BLU_REEF_IJ_L4);
        normalToL4PosMap.put(ReefFace.BLU_REEF_KL, ReefFace.BLU_REEF_KL_L4);



    }    

    public Pose2d getFinalTargetPose(boolean isAutoLeftAlign) {
        Pose2d currPoseEstimate = PoseEstimatorSubsystem.getInstance().getCurrentPose(); //current pose estimate from PoseEstimator
        ReefFace closestToCurrentFace = LineupMaster.getClosestReefFace(() -> currPoseEstimate); //current closest ReefFace
        
        Pose2d finalTargetPose = Pose2d.kZero;
        
        boolean isLeftAlignRequested = DriverStation.isAutonomous() ? isAutoLeftAlign : DriverXbox.getInstance().isLeftPovPressed();


        //call getAppropriateElevatorLineupOffset()

        //transform finalTargetPose by elevator offset

        //call getAppropriateWristLineupOffset()

        //transform finalTargetPose based on wrist rotation
        
        //transform finalTargetPose by robotTransform (?) - TEST THIS, check if needed or not


        return finalTargetPose;
    }

    public Pose2d getAppropriateElevatorLineupOffset(ReefFace nearestReefFace, boolean isLeftAlign) {
        //handle Elevator L4 vs L3, L2, L1 logic here
        boolean isElevatorStateL4 = (ElevatorSubsystem.kLastState == ElevatorSubsystem.State.kCoralL4) || 
                                    (ElevatorSubsystem.kLastState == ElevatorSubsystem.State.kCoralL4AutonScore) || 
                                    (ElevatorSubsystem.kLastState == ElevatorSubsystem.State.kCoralScoreL4);
        
        Pose2d appropriateElevatorPose = Pose2d.kZero;
        
        if(isElevatorStateL4 && normalToL4PosMap.containsKey(nearestReefFace)) {
            appropriateElevatorPose = (isLeftAlign) ? normalToL4PosMap.get(nearestReefFace).leftBranch : normalToL4PosMap.get(nearestReefFace).rightBranch;
        }
        
        return appropriateElevatorPose;
    }

    public Pose2d getAppropriateWristLineupOffset() {
        //handle right facing vs left facing wrist

        boolean isWristRightFacing = (RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight) ||
                                    (RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right) ||
                                    (RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist) || 
                                    (RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist) || 
                                    (RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist) || 
                                    (RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid) ||
                                    (RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid) ||
                                    (RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid);

        return Pose2d.kZero;
    }
}
