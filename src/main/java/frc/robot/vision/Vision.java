package frc.robot.vision;

import static frc.robot.vision.VisionConfig.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {


    /**
     * This code is a modified version of the example found in:
     * https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest
     * 
     * The modifications (should) allow it to run pose estimators using multiple cameras and detect objects
     */

    public class VisionEstimator {
        private final PhotonCamera photonCamera;
        private final Transform3d robotToCam;
        private final PhotonPoseEstimator poseEstimator;
        private Matrix<N3, N1> stdDevs;
        

        public VisionEstimator(PhotonCamera camName, Transform3d camLocation) {
            this.photonCamera = camName;
            this.robotToCam = camLocation;

            // declares a new photon pose estimator using the given parameters
            // also sets the fall back strategy to LOWEST_AMBIGUITY in case MULTI_TAG_PNP_ON_COPROCESSOR fails
            poseEstimator = new PhotonPoseEstimator(
                tagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                camLocation);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            }

            /**
             * The latest estimated robot pose on the field from vision data. This may be empty. This should
             * only be called once per loop.
             *
             * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
             * {@link getEstimationStdDevs}
             *
             * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
             *     used for estimation.
             */
            public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
                Optional<EstimatedRobotPose> visionEst = Optional.empty();
                for (var change : photonCamera.getAllUnreadResults()) {
                    visionEst = poseEstimator.update(change);
                    updateEstimationStdDevs(visionEst, change.getTargets());
                }
                return visionEst;
            }
            
            /**
             * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
             * deviations based on number of tags, estimation strategy, and distance from the tags.
             *
             * @param estimatedPose The estimated pose to guess standard deviations for.
             * @param targets All targets in this camera frame
             */
            private void updateEstimationStdDevs(
                    Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
                if (estimatedPose.isEmpty()) {
                    // No pose input. Default to single-tag std devs
                    stdDevs = singleTagStdDevs;

                } else {
                    // Pose present. Start running Heuristic
                    var estStdDevs = singleTagStdDevs;
                    int numTags = 0;
                    double avgDist = 0;

                    // Precalculation - see how many tags we found, and calculate an average-distance metric
                    for (var tgt : targets) {
                        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                        if (tagPose.isEmpty()) continue;
                        numTags++;
                        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(
                                    estimatedPose.get().estimatedPose.toPose2d().getTranslation()
                                    );
                    }
                    
                }

            }
        }
    }
