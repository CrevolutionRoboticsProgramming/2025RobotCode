package frc.robot.vision;

import static frc.robot.vision.VisionConfig.multiTagStdDevs;
import static frc.robot.vision.VisionConfig.singleTagStdDevs;
import static frc.robot.vision.VisionConfig.tagLayout;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionConfig.*;

/*
 * This code is a modified version of Photonvision's own example and 7028's vision code:
 * https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest
 * https://github.com/STMARobotics/frc-7028-2025
 */

public class Vision {

    private final PhotonCamera cam;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private static Vision[] mInstance;

    //Construct Vision Instance
    public Vision(String cameraName, Transform3d camTransform){
        cam = new PhotonCamera(cameraName);

        photonEstimator = new PhotonPoseEstimator(
            tagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            camTransform);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public static Vision[] getInstance() {
        if (mInstance == null) {
            mInstance = new Vision[VisionConfig.camNames.length];
            for (int i = 0; i < VisionConfig.camNames.length; i++){
                mInstance[i] = new Vision(VisionConfig.camNames[i], VisionConfig.robotToCamTransforms[i]);
            }
        }
        return mInstance;
    }

    /**
    * The latest estimated robot pose on the field from vision data. This may be empty. This should
    * only be called once per loop.
    *
    * <p>
    * Also includes updates for the standard deviations, which can (optionally) be retrieved with
    * {@link getEstimationStdDevs}
    *
    * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
    *         used for estimation.
    */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : cam.getAllUnreadResults()) {
          visionEst = photonEstimator.update(change);
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
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets){
        if (estimatedPose.isEmpty()){
            //No pose input. Default to single-tag std devs
            curStdDevs = singleTagStdDevs;
        
        } else {
            //Pose present, start Heuristic
            var estStdDevs = singleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            //Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()){
                    continue;
                }
                numTags++;
                avgDist += tagPose.get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0){
                //No tags visible. Default to single-tag std devs
                curStdDevs = singleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                  estStdDevs = multiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4) {
                  estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                  estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                }
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
    * Returns the latest standard deviations of the estimated pose from {@link
    * #getEstimatedGlobalPose()}, for use with {@link
    * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
    * only be used when there are targets visible.
    */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
}