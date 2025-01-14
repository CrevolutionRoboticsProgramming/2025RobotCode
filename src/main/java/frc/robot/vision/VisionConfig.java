package frc.robot.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConfig {
    
    //Creates camera names; ensure these all match with the correct camera on the Photonvison Dashboard
    public static final String centerPoseCamName = "Center_Pose_Cam";
    public static final String leftPoseCamName = "Left_Pose_Cam";
    public static final String rightPoseCamName = "Right_Pose_Cam";
    public static final String driveCamName = "Drive_Cam";


    //Creates all Photoncameras that will be used in pose estimator and commands
    public static PhotonCamera centerCam = new PhotonCamera(centerPoseCamName);
    public static PhotonCamera leftCam = new PhotonCamera(leftPoseCamName);
    public static PhotonCamera rightCam = new PhotonCamera(rightPoseCamName);
    public static PhotonCamera driveCam = new PhotonCamera(driveCamName);

    //Creates field layout for AprilTags
    public static AprilTagFields aprilTagField = AprilTagFields.k2025Reefscape;


    //TODO: config 
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