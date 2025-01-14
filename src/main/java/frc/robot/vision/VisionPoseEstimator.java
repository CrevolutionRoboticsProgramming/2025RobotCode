package frc.robot.vision;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionPoseEstimator extends SubsystemBase {
    
    public class PhotonRun implements Runnable {

        private final PhotonCamera photonCamera;
        private final Transform3d robotToCam;
        private final PhotonPoseEstimator poseEstimator;
        private final AtomicReference<EstimatedRobotPose> atomicRobotPose = new AtomicReference<EstimatedRobotPose>();

        public PhotonRun(PhotonCamera cam, Transform3d robotToCam) {
            //declares photoncamera, camera position, and prepares the pose estimator for initialization
            //photon camera not needed anymore apparently? needs testing
            this.photonCamera = cam;
            this.robotToCam = robotToCam;
            PhotonPoseEstimator photonEstimator = null;

            try {
                var layout = AprilTagFieldLayout.loadField(VisionConfig.aprilTagField);
                layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

                if (robotToCam != null){
                    photonEstimator = new PhotonPoseEstimator(layout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                    robotToCam);
                }

            } catch (Exception e) {
                DriverStation.reportError("Failed to load AprilTag field", e.getStackTrace());
                photonEstimator = null;
            }
            this.poseEstimator = photonEstimator;
        }


        @Override
        public void run() {
            if (poseEstimator != null && photonCamera != null){
                var photonResults = photonCamera.getAllUnreadResults();
                PhotonPipelineResult target = new PhotonPipelineResult();

                
            }
        }

        public EstimatedRobotPose getLatestEstimatedPose(){
            return atomicRobotPose.getAndSet(getLatestEstimatedPose());
        }

    }

}
