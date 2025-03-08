package frc.robot.vision;

import static edu.wpi.first.units.Units.Meters;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;

public class VisionConfig {
    
    // Creates camera names; ensure these all match with the correct camera on the Photonvison Dashboard
    public static final int TOTAL_CAMS = 2; //TODO: change to 4 where 4 cams are available
    public static final String[] CAM_NAMES = new String[] {"Left_Cam", "Right_Cam"}; //TODO: add center cam and drive cam

    //Camera Positions
    public static final Transform3d[] ROBOT_TO_CAM_TRANSFORMS = new Transform3d[] {
        //left cam
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-12.436), Units.inchesToMeters(11.677), Units.inchesToMeters(7.413)),
            new Rotation3d(0,Units.degreesToRadians(15),Units.degreesToRadians(-20))),
        //right cam
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-12.436), Units.inchesToMeters(-11.677), Units.inchesToMeters(7.413)), 
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
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

}