package frc.robot.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.drivetrain.TunerConstants;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorWrist;

public class VisionConfig {
    
    // Creates camera names; ensure these all match with the correct camera on the Photonvison Dashboard
    public static final int TOTAL_CAMS = 2; //TODO: change to 4 where 4 cams are available
    public static final String[] CAM_NAMES = new String[] {"Left_Cam", "Right_Cam"}; //TODO: add center cam and drive cam

    //Camera Positions
    public static final Transform3d[] ROBOT_TO_CAM_TRANSFORMS = new Transform3d[] {
        //left cam
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-11.882),Units.inchesToMeters(-11.020),Units.inchesToMeters(6.767)),
            new Rotation3d(0,Units.degreesToRadians(15),Units.degreesToRadians(160))),
        //right cam
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-11.882),Units.inchesToMeters(11.020),Units.inchesToMeters(6.767)), 
            new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(-160)))
    }; 

    // Creates field layout for AprilTags
    public static AprilTagFieldLayout TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Standard deviation of vision poses, this helps with correction or something idk thats what photon said
    // TODO: experiment with standard deviation values and set them to whatever gives the most correct pose
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8); // TODO: example values, change when testing
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1); //TODO: change values when testing

    public static final double AMBIGUITY_THRESHOLD = 0.2;
    public static final Distance SINGLE_TAG_DISTANCE_THRESHOLD = Meters.of(4.5);    

    public static final Distance FIELD_LENGTH = Meters.of(17.548);
    public static final Distance FIELD_WIDTH = Meters.of(8.052);

    public static final double FIELD_LENGTH_METERS = 17.548;
    public static final double FIELD_WIDTH_METERS = 8.052;

    public static boolean USE_VISION = true;


    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;


     /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(10));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = VecBuilder.fill(0.1, 0.1, 0.1);


    public static class AlignmentConfig {

        //Starting with same PID values as Auton
        public static final double XY_kP = 5.0;
        public static final double XY_kI = 0.0;
        public static final double XY_kD = 0.0;
    
        public static final double THETA_kP = 6.5;
        public static final double THETA_kI = 0.0;
        public static final double THETA_kD = 0.05;    
        
        public static final Distance DISTANCE_TOLERANCE = Inches.of(0.5);
        public static final Distance LATERAL_TOLERANCE = Inches.of(1.0);
        public static final double THETA_TOLERANCE = 0.03;

        public static final Distance ALIGNMENT_TOLERANCE = Inches.of(0.5); //inches
        public static final LinearVelocity MAX_ALIGN_TRANSLATION_VELOCITY = TunerConstants.kSpeedAt12Volts.div(8);
        public static final LinearAcceleration MAX_ALIGN_TRANSLATION_ACCELERATION = MetersPerSecondPerSecond.of(6.0);
        public static final AngularVelocity MAX_ALIGN_ANGULAR_VELOCITY = RotationsPerSecond.of(1.25).times(0.75);
        public static final AngularAcceleration MAX_ALIGN_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(6.0 * Math.PI);
        
    }

    /*Red Alliance */
    private static final double acutalX = 5.575;

    /*Blue Alliance */
    // private static final double acutalX = 5.1;

    private static final double x = 0.5 * acutalX;
    private static final double y = (Math.sqrt(3)/2) * acutalX;

    
    public enum ReefFace {
        // IMPORTANT: Fudge factors are always positive and should be in meters (use the Units.inchesToMeters() method)

        // Blue Reef
        BLU_REEF_AB_L4(18, 3.657600 - Units.inchesToMeters(acutalX), 4.025900, 180.0, null, null),
        BLU_REEF_CD_L4(17, 4.073906 - Units.inchesToMeters(x), 3.306318 - Units.inchesToMeters(y), 240.0, null, null),
        BLU_REEF_EF_L4(22, 4.904740 + Units.inchesToMeters(x), 3.306318 - Units.inchesToMeters(y), 300.0, null, null),
        BLU_REEF_GH_L4(21, 5.321046 + Units.inchesToMeters(acutalX), 4.025900, 0.0, null, null),
        BLU_REEF_IJ_L4(20, 4.904740 + Units.inchesToMeters(x), 4.745482 + Units.inchesToMeters(y), 60.0, null, null),
        BLU_REEF_KL_L4(19, 4.073906 - Units.inchesToMeters(x), 4.745482 + Units.inchesToMeters(y), 120.0, null, null),
        RED_REEF_AB_L4(7, 13.890498 + Units.inchesToMeters(acutalX), 4.025900, 0.0, null, null),
        RED_REEF_CD_L4(8, 13.474446 + Units.inchesToMeters(x), 4.745482 + Units.inchesToMeters(y), 60., null, null),
        RED_REEF_EF_L4(9, 12.643358 - Units.inchesToMeters(x), 4.745482 + Units.inchesToMeters(y), 120.0, null, null),
        RED_REEF_GH_L4(10, 12.227306 - Units.inchesToMeters(acutalX), 4.025900, 180.0, null, null),
        RED_REEF_IJ_L4(11, 12.643358 - Units.inchesToMeters(x), 3.306318 - Units.inchesToMeters(y), 240.0, null, null),
        RED_REEF_KL_L4(6, 13.474446 + Units.inchesToMeters(x), 3.306318 - Units.inchesToMeters(y), 300.0, null, null),
        BLU_REEF_AB(18, 3.657600, 4.025900, 180.0, null, null),
        BLU_REEF_CD(17, 4.073906, 3.306318, 240.0, null, null),
        BLU_REEF_EF(22, 4.904740, 3.306318, 300.0, null, null),
        BLU_REEF_GH(21, 5.321046, 4.025900, 0.0, null, null),
        BLU_REEF_IJ(20, 4.904740, 4.745482, 60.0, null, null),
        BLU_REEF_KL(19, 4.073906, 4.745482, 120.0, null, null),
        RED_REEF_AB(7, 13.890498, 4.025900, 0.0, null, null),
        RED_REEF_CD(8, 13.474446, 4.745482, 60., null, null),
        RED_REEF_EF(9, 12.643358, 4.745482, 120.0, null, null),
        RED_REEF_GH(10, 12.227306, 4.025900, 180.0, null, null),
        RED_REEF_IJ(11, 12.643358, 3.306318, 240.0, null, null),
        RED_REEF_KL(6, 13.474446, 3.306318, 300.0, null, null);

        


        public final Double leftBranchFudgeTransform;
        public final Double rightBranchFudgeTransform;
        public final Pose2d leftBranch;
        public final Pose2d rightBranch;
        public final Pose2d AprilTag;
        public final int aprilTagID;
        public final double aprilTagX;
        public final double aprilTagY;
        public final double aprilTagTheta;

        //AT stands for AprilTag
        @SuppressWarnings("unused")
        private ReefFace(int aprilTagID, double aprilTagX, double aprilTagY, double aprilTagTheta, Double leftBranchFudgeTransform, Double rightBranchFudgeTransform) {
            // if (RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight ||
            //     RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
            //     RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid ||
            //     RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid) {
                
            //     this.aprilTagID = aprilTagID;
            //     this.AprilTag = new Pose2d(aprilTagX, aprilTagY, Rotation2d.fromDegrees(aprilTagTheta));
            //     this.leftBranchFudgeTransform = leftBranchFudgeTransform;
            //     this.rightBranchFudgeTransform = rightBranchFudgeTransform;

            //     if (this.leftBranchFudgeTransform == null) {
            //         this.leftBranch = AprilTag.transformBy(leftBranchTransformRightWrist);
            //     } else {
            //         this.leftBranch = AprilTag.transformBy(new Transform2d(0.0, -this.leftBranchFudgeTransform, Rotation2d.kZero));
            //     }
                
            //     if (this.rightBranchFudgeTransform == null) {
            //         this.rightBranch = AprilTag.transformBy(rightBranchTransformRightWrist);
            //     } else {
            //         this.rightBranch = AprilTag.transformBy(new Transform2d(0.0, this.rightBranchFudgeTransform, Rotation2d.kZero));
            //     }
            // } 
            // else {
            //     this.aprilTagID = aprilTagID;
            //     this.AprilTag = new Pose2d(aprilTagX, aprilTagY, Rotation2d.fromDegrees(aprilTagTheta));
            //     this.leftBranchFudgeTransform = leftBranchFudgeTransform;
            //     this.rightBranchFudgeTransform = rightBranchFudgeTransform;

            //     if (this.leftBranchFudgeTransform == null) {
            //         this.leftBranch = AprilTag.transformBy(leftBranchTransformLeftWrist);
            //     } else {
            //         this.leftBranch = AprilTag.transformBy(new Transform2d(0.0, -this.leftBranchFudgeTransform, Rotation2d.kZero));
            //     }
                
            //     if (this.rightBranchFudgeTransform == null) {
            //         this.rightBranch = AprilTag.transformBy(rightBranchTransformLeftWrist);
            //     } else {
            //         this.rightBranch = AprilTag.transformBy(new Transform2d(0.0, this.rightBranchFudgeTransform, Rotation2d.kZero));
            //     }
            // }
            this.aprilTagID = aprilTagID;
            this.aprilTagX = aprilTagX;
            this.aprilTagY = aprilTagY;
            this.aprilTagTheta = aprilTagTheta;
            this.AprilTag = new Pose2d(aprilTagX, aprilTagY, Rotation2d.fromDegrees(aprilTagTheta));
            this.leftBranchFudgeTransform = leftBranchFudgeTransform;
            this.rightBranchFudgeTransform = rightBranchFudgeTransform;

            this.leftBranch = AprilTag;
            this.rightBranch = AprilTag;

            // if (this.leftBranchFudgeTransform == null) {
            //     this.leftBranch = AprilTag.transformBy(leftBranchTransformLeftWrist);
            // } else {
            //     this.leftBranch = AprilTag.transformBy(new Transform2d(0.0, -this.leftBranchFudgeTransform, Rotation2d.kZero));
            // }
            
            // if (this.rightBranchFudgeTransform == null) {
            //     this.rightBranch = AprilTag.transformBy(rightBranchTransformLeftWrist);
            // } else {
            //     this.rightBranch = AprilTag.transformBy(new Transform2d(0.0, this.rightBranchFudgeTransform, Rotation2d.kZero));
            // }

        }

    }

    
}