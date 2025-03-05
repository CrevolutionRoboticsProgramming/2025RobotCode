package frc.robot.vision;

import static frc.robot.vision.VisionConfig.camNames;
import static frc.robot.vision.VisionConfig.robotToCamTransforms;

import java.util.Arrays;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivetrain.CommandSwerveDrivetrain;


public class SwerveVisionEstimator extends SubsystemBase{

    private final Vision[] visions;
    private static SwerveVisionEstimator mInstance;
    private final Supplier<SwerveModulePosition[]> modSupplier;
    private final Supplier<Rotation2d> rotationSupplier;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d = new Field2d();
    

    public SwerveVisionEstimator(
        Supplier<Rotation2d> rotSup, Supplier<SwerveModulePosition[]> modSup){
            this.modSupplier = modSup;
            this.rotationSupplier = rotSup;

            poseEstimator = new SwerveDrivePoseEstimator(
                //DriveConstants.swerveKinematics
                null,
                rotationSupplier.get(),
                modSupplier.get(),
                new Pose2d());

            visions = new Vision[camNames.length];
            for (int i = 0; i < camNames.length; i++){
                visions[i] = new Vision(camNames[i], robotToCamTransforms[i]);
            }
        }
    
    public static SwerveVisionEstimator getInstance(){
        if (mInstance == null){
            final var drivetrain = CommandSwerveDrivetrain.getInstance();
            //mInstance = new SwerveVisionEstimator(drivetrain::getGyroYaw, drivetrain::getModulePositions);
            mInstance = new SwerveVisionEstimator(() -> Rotation2d.fromDegrees(CommandSwerveDrivetrain.mGyro.getYaw().getValueAsDouble()), null);
        }
        return mInstance;
    }

    public Pose2d getCurrrentPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public void estimatorUpdate(){
        Arrays.stream(visions).forEach(vision -> {
            var visionEst = vision.getEstimatedGlobalPose();
            
            visionEst.ifPresent(est -> {
                var estStdDevs = vision.getEstimationStdDevs();
                
                poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
        });
    }

    private String getFormattedPose(){
        var pose = getCurrrentPose();
        return String.format("(%.3f, %.3f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
    }

    @Override
    public void periodic(){
        // //Update swerve pose estimator using drivetrain
        poseEstimator.update(rotationSupplier.get(), modSupplier.get());
        //Add vision measurements to swerve pose estimator
        estimatorUpdate();

        //log to dashboard
        var dashboardPose = poseEstimator.getEstimatedPosition();
        SmartDashboard.putString("Estimated Pose", getFormattedPose());
        SmartDashboard.putData(field2d);
        field2d.setRobotPose(getCurrrentPose());
    // }
    }
}