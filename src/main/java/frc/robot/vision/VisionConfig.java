package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConfig {
    
    public static final String centerPoseCam = "Center_Pose_Cam";
    public static final String leftPoseCam = "Left_Pose_Cam";
    public static final String rightPoseCam = "Right_Pose_Cam";
    public static final String driveCam = "Drive_Cam";

    //TODO: when photonlib is added, create photon cameras with names
    

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


        //APRIL TAG ID COORDS
        //XYZ Origin (0, 0) is established in the bottom left corner of the field

        public static final Transform3d aprilTagID1 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID2 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID3 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID4 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID5 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID6 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID7 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID8 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID9 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID10 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID11 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID12 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID13 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID14 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID15 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID16 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID17 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID18 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID19 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID20 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID21 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );

        public static final Transform3d aprilTagID22 = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)) 
        );
}
