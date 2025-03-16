package frc.robot.vision.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.vision.VisionConfig;
import frc.robot.drivetrain.*;
import frc.robot.vision.PoseEstimatorSubsystem;

public class LineupCommand extends Command {
    private static final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_TRANSLATION_VELOCITY.in(MetersPerSecond),
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_TRANSLATION_ACCELERATION.in(MetersPerSecondPerSecond));
            
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_ANGULAR_VELOCITY.in(RadiansPerSecond),
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));


    private static final ProfiledPIDController xDistanceController = new ProfiledPIDController(
                                        VisionConfig.AlignmentConfig.X_kP,
                                        VisionConfig.AlignmentConfig.X_kI,
                                        VisionConfig.AlignmentConfig.X_kD,
                                        TRANSLATION_CONSTRAINTS);

    private static final ProfiledPIDController yDistanceController = new ProfiledPIDController(
                                        VisionConfig.AlignmentConfig.Y_kP,
                                        VisionConfig.AlignmentConfig.Y_kI,
                                        VisionConfig.AlignmentConfig.Y_kD,
                                        TRANSLATION_CONSTRAINTS);

    private static final ProfiledPIDController thetaController = new ProfiledPIDController(
                                        VisionConfig.AlignmentConfig.THETA_kP,
                                        VisionConfig.AlignmentConfig.THETA_kI,
                                        VisionConfig.AlignmentConfig.THETA_kD,
                                        THETA_CONSTRAINTS);
    
    private static final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
                                        .withDriveRequestType(DriveRequestType.Velocity)
                                        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        

    private static Pose2d currentPose;

    public LineupCommand() {
        //set tolerances of all PID controllers
        xDistanceController.setTolerance(VisionConfig.AlignmentConfig.DISTANCE_TOLERANCE.in(Meters));
        yDistanceController.setTolerance(VisionConfig.AlignmentConfig.LATERAL_TOLERANCE.in(Meters));
        thetaController.setTolerance(VisionConfig.AlignmentConfig.THETA_TOLERANCE);

        currentPose = PoseEstimatorSubsystem.getInstance().getCurrentPose();

        addRequirements(CommandSwerveDrivetrain.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

