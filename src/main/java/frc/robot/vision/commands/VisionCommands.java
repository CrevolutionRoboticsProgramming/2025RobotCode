package frc.robot.vision.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.vision.PoseEstimatorSubsystem;
import frc.robot.vision.VisionConfig;

public class VisionCommands {
    
    public Command executeLineup() {
        Pose2d estimatedPose = PoseEstimatorSubsystem.getInstance().getCurrentPose();

        //add ProfiledPID controllers for Translation and Rotation
        final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    VisionConfig.AlignmentConfig.MAX_ALIGN_TRANSLATION_VELOCITY.in(MetersPerSecond),
                    VisionConfig.AlignmentConfig.MAX_ALIGN_TRANSLATION_ACCELERATION.in(MetersPerSecondPerSecond));
            
        final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    VisionConfig.AlignmentConfig.MAX_ALIGN_ANGULAR_VELOCITY.in(RadiansPerSecond),
                    VisionConfig.AlignmentConfig.MAX_ALIGN_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));


        final ProfiledPIDController xDistanceController = new ProfiledPIDController(
            VisionConfig.AlignmentConfig.X_kP,
            VisionConfig.AlignmentConfig.X_kI,
            VisionConfig.AlignmentConfig.X_kD,
            TRANSLATION_CONSTRAINTS);

        final ProfiledPIDController yDistanceController = new ProfiledPIDController(
            VisionConfig.AlignmentConfig.Y_kP,
            VisionConfig.AlignmentConfig.Y_kI,
            VisionConfig.AlignmentConfig.Y_kD,
            TRANSLATION_CONSTRAINTS);

        final ProfiledPIDController thetaController = new ProfiledPIDController(
            VisionConfig.AlignmentConfig.THETA_kP,
            VisionConfig.AlignmentConfig.THETA_kI,
            VisionConfig.AlignmentConfig.THETA_kD,
            THETA_CONSTRAINTS);
        
        final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
            

        return new WaitCommand(0);
    }



}
