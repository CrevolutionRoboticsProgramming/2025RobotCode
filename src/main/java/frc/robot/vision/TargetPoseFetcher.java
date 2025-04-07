package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.VisionConfig.ReefFace;

public class TargetPoseFetcher {

    public TargetPoseFetcher() {}    

    public Pose2d getFinalTargetPose() {
        Pose2d currPoseEstimate = PoseEstimatorSubsystem.getInstance().getCurrentPose();
        ReefFace closestToCurrentFace = LineupMaster.getClosestReefFace(() -> currPoseEstimate);
        

        return Pose2d.kZero;
    }
}
