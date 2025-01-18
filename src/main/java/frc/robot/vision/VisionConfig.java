package frc.robot.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConfig {
    
    // Creates camera names; ensure these all match with the correct camera on the Photonvison Dashboard
    // public static final String centerPoseCamName = "Center_Pose_Cam";
    // public static final String leftPoseCamName = "Left_Pose_Cam";
    // public static final String rightPoseCamName = "Right_Pose_Cam";
    // public static final String driveCamName = "Drive_Cam";


    // Creates all Photoncameras that will be used in pose estimator and commands
    public static final PhotonCamera centerCam = new PhotonCamera("Center_Pose_Cam");
    public static final PhotonCamera leftCam = new PhotonCamera("Left_Pose_Cam");
    public static final PhotonCamera rightCam = new PhotonCamera("Right_Pose_Cam");
    public static final PhotonCamera driveCam = new PhotonCamera("Drive_Cam");

    // Creates field layout for AprilTags
    public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // Standard deviation of vision poses, this helps with correction or something idk thats what photon said
    // TODO: experiment with standard deviation values and set them to whatever gives the most correct pose
    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8); // TODO: example values, change when testing
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1); //TODO: change values when testing

    // TODO: config 
    public static final Transform3d robotToCenterCam = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );
    
    public static final Transform3d robotToLeftCam = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

    public static final Transform3d robotToRightCam = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );
    
    public static final Transform3d robotToDriveCam =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0))
        );   
}