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

public class VisionConfig {
    
    // Creates camera names; ensure these all match with the correct camera on the Photonvison Dashboard
    public static final int TOTAL_CAMS = 2; //TODO: change to 4 where 4 cams are available
    public static final String[] CAM_NAMES = new String[] {"Left_Cam", "Right_Cam"}; //TODO: add center cam and drive cam

    //Camera Positions
    public static final Transform3d[] ROBOT_TO_CAM_TRANSFORMS = new Transform3d[] {
        //left cam
        new Transform3d(
            new Translation3d(Units.inchesToMeters(11.882),Units.inchesToMeters(11.020),Units.inchesToMeters(6.767)),
            new Rotation3d(0,Units.degreesToRadians(15),Units.degreesToRadians(-20))),
        //right cam
        new Transform3d(
            new Translation3d(Units.inchesToMeters(11.882),Units.inchesToMeters(-11.020),Units.inchesToMeters(6.767)), 
            new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(20)))
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
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill(.05, .05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = VecBuilder.fill(0.025, 0.025, Units.degreesToRadians(2.5));


    public static class AlignmentConfig {
        public static final double THETA_kP = 3.0;
        public static final double THETA_kI = 0.0;
        public static final double THETA_kD = 0.0;
    
        public static final double X_kP = 5.0;
        public static final double X_kI = 0.0;
        public static final double X_kD = 0.0;
    
        public static final double Y_kP = 5.0;
        public static final double Y_kI = 0.0;
        public static final double Y_kD = 0.0;
        
        public static final Distance DISTANCE_TOLERANCE = Inches.of(0.5);
        public static final Distance LATERAL_TOLERANCE = Inches.of(1.0);
        public static final double THETA_TOLERANCE = 0.03;

        public static final Distance ALIGNMENT_TOLERANCE = Inches.of(0.5); //inches
        public static final LinearVelocity MAX_ALIGN_TRANSLATION_VELOCITY = TunerConstants.kSpeedAt12Volts.div(2);
        public static final LinearAcceleration MAX_ALIGN_TRANSLATION_ACCELERATION = MetersPerSecondPerSecond.of(6.0);
        public static final AngularVelocity MAX_ALIGN_ANGULAR_VELOCITY = RotationsPerSecond.of(1.25).times(0.75);
        public static final AngularAcceleration MAX_ALIGN_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(6.0 * Math.PI);
    
        //April Tag IDs
        public static final double id1 = 1;
        public static final double id2 = 2;
        public static final double id3 = 3;
        public static final double id4 = 4;
        public static final double id5 = 5;
        public static final double id6 = 6;
        public static final double id7 = 7;
        public static final double id8 = 8;
        public static final double id9 = 9;
        public static final double id10 = 10;
        public static final double id11 = 11;
        public static final double id12 = 12;
        public static final double id13 = 13;
        public static final double id14 = 14;
        public static final double id15 = 15;
        public static final double id16 = 16;
        public static final double id17 = 17;
        public static final double id18 = 18;
        public static final double id19 = 19;
        public static final double id20 = 20;
        public static final double id21 = 21;
        public static final double id22 = 22;

        //Joey's Pose Constants for the Reef Locations
        public static final Pose2d Error = new Pose2d(6, 6, Rotation2d.fromDegrees(0));

        public static final Pose2d Ablue = new Pose2d(3.180, 4.175, Rotation2d.fromDegrees(0)); 
        public static final Pose2d Bblue = new Pose2d(3.180, 3.850, Rotation2d.fromDegrees(0));
        public static final Pose2d Cblue = new Pose2d(3.685, 2.975, Rotation2d.fromDegrees(60));
        public static final Pose2d Dblue = new Pose2d(3.975, 2.825, Rotation2d.fromDegrees(60));
        public static final Pose2d Eblue = new Pose2d(5.000, 2.825, Rotation2d.fromDegrees(120));
        public static final Pose2d Fblue = new Pose2d(5.285, 2.975, Rotation2d.fromDegrees(120));
        public static final Pose2d Gblue = new Pose2d(5.8, 3.850, Rotation2d.fromDegrees(180));
        public static final Pose2d Hblue = new Pose2d(5.8, 4.175, Rotation2d.fromDegrees(180));
        public static final Pose2d Iblue = new Pose2d(5.285, 5.075, Rotation2d.fromDegrees(240));
        public static final Pose2d Jblue = new Pose2d(5.000, 5.230, Rotation2d.fromDegrees(240));
        public static final Pose2d Kblue = new Pose2d(3.975, 5.230, Rotation2d.fromDegrees(300));
        public static final Pose2d Lblue = new Pose2d(3.685, 5.075, Rotation2d.fromDegrees(300));

        public static final double fieldFlip = 17.5;
        public static final double fieldFlipy = 8;

        public static final Pose2d Ared = new Pose2d(fieldFlip - 3.180, fieldFlipy - 4.175, Rotation2d.fromDegrees(180));
        public static final Pose2d Bred = new Pose2d(fieldFlip - 3.180, fieldFlipy - 3.850, Rotation2d.fromDegrees(180));
        public static final Pose2d Cred = new Pose2d(fieldFlip - 3.685, fieldFlipy - 2.975, Rotation2d.fromDegrees(-120));
        public static final Pose2d Dred = new Pose2d(fieldFlip - 3.975, fieldFlipy - 2.825, Rotation2d.fromDegrees(-120));
        public static final Pose2d Ered = new Pose2d(fieldFlip - 5.000, fieldFlipy - 2.825, Rotation2d.fromDegrees(-60));
        public static final Pose2d Fred = new Pose2d(fieldFlip - 5.285, fieldFlipy - 2.975, Rotation2d.fromDegrees(-60));
        public static final Pose2d Gred = new Pose2d(fieldFlip - 5.8, fieldFlipy - 3.850, Rotation2d.fromDegrees(0));
        public static final Pose2d Hred = new Pose2d(fieldFlip - 5.8, fieldFlipy - 4.175, Rotation2d.fromDegrees(0));
        public static final Pose2d Ired = new Pose2d(fieldFlip - 5.285, fieldFlipy - 5.075, Rotation2d.fromDegrees(-300));
        public static final Pose2d Jred = new Pose2d(fieldFlip - 5.000, fieldFlipy - 5.230, Rotation2d.fromDegrees(-300));
        public static final Pose2d Kred = new Pose2d(fieldFlip - 3.975, fieldFlipy - 5.230, Rotation2d.fromDegrees(-240));
        public static final Pose2d Lred = new Pose2d(fieldFlip - 3.685, fieldFlipy - 5.075, Rotation2d.fromDegrees(-240));


                //TODO: MIGHT NEED TO PLAY AROUND WITH ALL THE POSES BELOW

        // /** Pose of the robot relative to a reef branch for scoring coral on L4 */
        // public static final Transform2d RELATIVE_SCORING_POSE_CORAL_L4 = new Transform2d(
        //     Units.inchesToMeters(-40),
        //     Units.inchesToMeters(12),
        //     Rotation2d.fromDegrees(-90));
        // /** Pose of the robot relative to a reef branch for scoring coral on L3 */
        // public static final Transform2d RELATIVE_SCORING_POSE_CORAL_L3 = new Transform2d(
        //     Units.inchesToMeters(-40),
        //     Units.inchesToMeters(12),
        //     Rotation2d.fromDegrees(-90));

        // /** Pose of the robot relative to a reef branch for scoring coral on L2 */
        // public static final Transform2d RELATIVE_SCORING_POSE_CORAL_L2 = new Transform2d(
        //     Units.inchesToMeters(-40),
        //     Units.inchesToMeters(12),
        //     Rotation2d.fromDegrees(90));

        // /** Pose of the robot relative to the reef trough for scoring coral on L1 */
        // public static final Transform2d RELATIVE_SCORING_POSE_CORAL_L1 = new Transform2d(
        //     Units.inchesToMeters(-20),
        //     Units.inchesToMeters(-8),
        //     Rotation2d.fromDegrees(90));
        
        // // spotless:off
        // /* The reef branches are in the arrays like this:
        // *    ----------------------------------------
        // *    |     5  / \ 6      |     11 / \ 0     |
        // *    B    4 /     \ 7    |   10 /     \ 1   |
        // *    L   3 |       | 8   |   9 |       | 2  R
        // * +X U   2 |       | 9   |   8 |       | 3  E
        // *    E    1 \     / 10   |    7 \     / 4   D
        // *    |      0 \ / 11     |      6 \ / 5     |
        // *    |___________________|__________________|
        // * (0, 0)               +Y
        // */
        // // spotless:on
        // /**
        //  * Poses of the right branches on the blue reef. Translation is the branch pipe base, rotation is pointing toward
        //  * reef center.
        //  */
        // public static final List<Pose2d> REEF_BRANCH_POSES_BLUE_RIGHT = Stream
        //     .of(
        //         new Pose2d(4.347746, 3.467, Rotation2d.fromDegrees(60)), // 0
        //         new Pose2d(3.942648, 3.840490, Rotation2d.fromDegrees(0)), // 2
        //         new Pose2d(4.062584, 4.398912, Rotation2d.fromDegrees(-60)), // 4
        //         new Pose2d(4.588763, 4.542161, Rotation2d.fromDegrees(-120)), // 6
        //         new Pose2d(4.98, 4.215, Rotation2d.fromDegrees(180)), // 8
        //         new Pose2d(4.873353, 3.632614, Rotation2d.fromDegrees(120))) // 10
        //     .collect(Collectors.toUnmodifiableList());

        // /**
        //  * Poses of the left branches on the blue reef. Translation is the branch pipe base, rotation is pointing toward
        //  * reef center.
        //  */
        // public static final List<Pose2d> REEF_BRANCH_POSES_BLUE_LEFT = Stream
        //     .of(
        //         new Pose2d(4.062584, 3.630770, Rotation2d.fromDegrees(60)), // 1
        //         new Pose2d(3.942648, 4.169106, Rotation2d.fromDegrees(0)), // 3
        //         new Pose2d(4.347175, 4.515, Rotation2d.fromDegrees(-60)), // 5
        //         new Pose2d(4.873926, 4.378820, Rotation2d.fromDegrees(-120)), // 7
        //         new Pose2d(4.994328, 3.841097, Rotation2d.fromDegrees(180)), // 9
        //         new Pose2d(4.589334, 3.466500, Rotation2d.fromDegrees(120)))// 11
        //     .collect(Collectors.toUnmodifiableList());

        // /**
        //  * Poses of the right branches on the red reef. Translation is the branch pipe base, rotation is pointing toward
        //  * reef center.
        //  */
        // public static final List<Pose2d> REEF_BRANCH_POSES_RED_RIGHT = Stream
        //     .of(
        //         new Pose2d(13.200254, 4.585000, Rotation2d.fromDegrees(-120)), // 0
        //         new Pose2d(13.605352, 4.211510, Rotation2d.fromDegrees(-180)), // 2
        //         new Pose2d(13.485416, 3.653088, Rotation2d.fromDegrees(120)), // 4
        //         new Pose2d(12.959237, 3.509839, Rotation2d.fromDegrees(60)), // 6
        //         new Pose2d(12.568000, 3.837000, Rotation2d.fromDegrees(0)), // 8
        //         new Pose2d(12.598000, 4.292000, Rotation2d.fromDegrees(-60))) // 10
        //     .collect(Collectors.toUnmodifiableList());

        // /**
        //  * Poses of the left branches on the red reef. Translation is the branch pipe base, rotation is pointing toward reef
        //  * center.
        //  */
        // public static final List<Pose2d> REEF_BRANCH_POSES_RED_LEFT = Stream
        //     .of(
        //         new Pose2d(13.485416, 4.421230, Rotation2d.fromDegrees(-120)), // 1
        //         new Pose2d(13.605352, 3.882894, Rotation2d.fromDegrees(-180)), // 3
        //         new Pose2d(13.200825, 3.537000, Rotation2d.fromDegrees(120)), // 5
        //         new Pose2d(12.674074, 3.673180, Rotation2d.fromDegrees(60)), // 7
        //         new Pose2d(12.553672, 4.210903, Rotation2d.fromDegrees(0)), // 9
        //         new Pose2d(12.958666, 4.585500, Rotation2d.fromDegrees(-60)))// 11
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L4 left branches on the red alliance */
        // public static final List<Pose2d> REEF_L4_SCORE_POSES_RED_LEFT = REEF_BRANCH_POSES_RED_LEFT.stream()
        // .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        // .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L4 right branches on the red alliance */
        // public static final List<Pose2d> REEF_L4_SCORE_POSES_RED_RIGHT = REEF_BRANCH_POSES_RED_RIGHT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L4 left branches on the blue alliance */
        // public static final List<Pose2d> REEF_L4_SCORE_POSES_BLUE_LEFT = REEF_BRANCH_POSES_BLUE_LEFT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L4 right branches on the blue alliance */
        // public static final List<Pose2d> REEF_L4_SCORE_POSES_BLUE_RIGHT = REEF_BRANCH_POSES_BLUE_RIGHT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L3 left branch on the red alliance */
        // public static final List<Pose2d> REEF_L3_SCORE_POSES_RED_LEFT = REEF_BRANCH_POSES_RED_LEFT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L3 right branch on the red alliance */
        // public static final List<Pose2d> REEF_L3_SCORE_POSES_RED_RIGHT = REEF_BRANCH_POSES_RED_RIGHT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L3 left branch on the blue alliance */
        // public static final List<Pose2d> REEF_L3_SCORE_POSES_BLUE_LEFT = REEF_BRANCH_POSES_BLUE_LEFT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L3 right branch on the blue alliance */
        // public static final List<Pose2d> REEF_L3_SCORE_POSES_BLUE_RIGHT = REEF_BRANCH_POSES_BLUE_RIGHT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L2 left branch on the red alliance */
        // public static final List<Pose2d> REEF_L2_SCORE_POSES_RED_LEFT = REEF_BRANCH_POSES_RED_LEFT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L2))
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L2 right branch on the red alliance */
        // public static final List<Pose2d> REEF_L2_SCORE_POSES_RED_RIGHT = REEF_BRANCH_POSES_RED_RIGHT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L2))
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L2 left branch on the blue alliance */
        // public static final List<Pose2d> REEF_L2_SCORE_POSES_BLUE_LEFT = REEF_BRANCH_POSES_BLUE_LEFT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L2))
        //     .collect(Collectors.toUnmodifiableList());

        // /** Poses of the robot for scoring on L2 right on the blue alliance */
        // public static final List<Pose2d> REEF_L2_SCORE_POSES_BLUE_RIGHT = REEF_BRANCH_POSES_BLUE_RIGHT.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L2))
        //     .collect(Collectors.toUnmodifiableList());

        // // spotless:off
        // /* The reef level 1 positions are in the arrays like this:
        // *    ----------------------------------------
        // *    |        / \ 3      |        / \ 0     |
        // *    B    2 /     \      |    5 /     \     |
        // *    L     |       | 4   |     |       | 1  R
        // * +X U   1 |       |     |   4 |       |    E
        // *    E      \     / 5    |      \     / 2   D
        // *    |      0 \ /        |      3 \ /       |
        // *    |___________________|__________________|
        // * (0, 0)               +Y
        // */
        // // spotless:on
        // public static final List<Pose2d> REEF_L1_POSES_BLUE = Stream
        // .of(
        //     new Pose2d(3.66, 3.540630, Rotation2d.fromDegrees(-30)),
        //     new Pose2d(3.66, 4.496321, Rotation2d.fromDegrees(-90)),
        //     new Pose2d(4.480743, 4.981, Rotation2d.fromDegrees(-150)),
        //     new Pose2d(5.312527, 4.511019, Rotation2d.fromDegrees(150)),
        //     new Pose2d(5.321074, 3.555505, Rotation2d.fromDegrees(90)),
        //     new Pose2d(4.497811, 3.070409, Rotation2d.fromDegrees(30)))
        // .collect(Collectors.toUnmodifiableList());

        // public static final List<Pose2d> REEF_L1_POSES_RED = Stream
        //     .of(
        //         new Pose2d(13.89, 4.51, Rotation2d.fromDegrees(150)),
        //         new Pose2d(13.89, 3.56, Rotation2d.fromDegrees(90)),
        //         new Pose2d(13.07, 3.07, Rotation2d.fromDegrees(30)),
        //         new Pose2d(12.24, 3.54, Rotation2d.fromDegrees(-30)),
        //         new Pose2d(12.23, 4.50, Rotation2d.fromDegrees(-90)),
        //         new Pose2d(13.05, 4.98, Rotation2d.fromDegrees(-150)))
        //     .collect(Collectors.toUnmodifiableList());

        // public static final List<Pose2d> REEF_L1_SCORE_POSES_BLUE = REEF_L1_POSES_BLUE.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L1))
        //     .collect(Collectors.toUnmodifiableList());

        // public static final List<Pose2d> REEF_L1_SCORE_POSES_RED = REEF_L1_POSES_RED.stream()
        //     .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L1))
        //     .collect(Collectors.toUnmodifiableList());

        // public static final Distance DISTANCE_TARGET_L4 = Meters.of(0.34);
        // public static final Distance DISTANCE_TARGET_L3 = Meters.of(0.34);

        // public static final Distance LATERAL_TARGET_L3_LEFT = Meters.of(0.05);
        // public static final Distance LATERAL_TARGET_L3_RIGHT = Meters.of(0.02);

        // public static final Distance LATERAL_TARGET_L4_LEFT = Meters.of(0.05);
        // public static final Distance LATERAL_TARGET_L4_RIGHT = Meters.of(0.02);
    }
}