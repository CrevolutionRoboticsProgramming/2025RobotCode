package frc.robot.drivetrain.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.CommandSwerveDrivetrain;

public class DriveAndHoldAngle extends Command{
    private static class Settings {
        static final double kP = 5.0;
        static final double kI = 0.0;
        static final double kD = 1.5;
        
        static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(360.0);
        static final Rotation2d kAllowedError = Rotation2d.fromDegrees(1.0);
    }

    private final CommandSwerveDrivetrain drivetrain;
    private Rotation2d deltaTheta;
    private Supplier<Translation2d> translationSupplier;

    private final PIDController pidController;
    final DriverXbox driver;

    private Rotation2d targetAngle;

    public DriveAndHoldAngle(Supplier<Translation2d> translationSupplier) {
        drivetrain = CommandSwerveDrivetrain.getInstance();
        this.deltaTheta = null;
        this.translationSupplier = translationSupplier;

        pidController = new PIDController(Settings.kP, Settings.kI, Settings.kD);

        driver = DriverXbox.getInstance();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Pose2d goalPose = null;
        /*Add Pose Estimator When Working */
        // final var mPoseEstimator = PoseEstimatorSubsystem.getInstance();
        // final var robotPose = mPoseEstimator.getCurrentPose();
        // Optional<Alliance> ally = DriverStation.getAlliance();
        // if (ally.isPresent()) {
        //     if (ally.get() == Alliance.Blue) {
        //         if (robotPose.getY() < 3.0) {
        //             /*Towards far side Goal Pose */
        //             goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        //         } else {
        //             /*Goal Pose for Close Side Goal (0,0) */
        //             goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        //         }    
        //     }
        //     if (ally.get() == Alliance.Red) {
        //         if (robotPose.getY() < 3.0) {
        //             /*Towards far side Goal Pose */
        //             goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        //         } else {
        //             /*Goal Pose for Close Side Goal (0,0) */
        //             goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        //         }    
        //     }
        // }
        // final var startingAngle = robotPose.getRotation();
        // final var endAngle = goalPose.getTranslation().minus(robotPose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0));
        // deltaTheta = endAngle.minus(startingAngle);

        // targetAngle = Rotation2d.fromDegrees(drivetrain.mGyro.getYaw().getValueAsDouble() + deltaTheta.getDegrees());

        // final var currentAngle = drivetrain.mGyro.getYaw().getValueAsDouble();
        // final var requestedAngularVelocity = Rotation2d.fromDegrees(MathUtil.clamp(
        //     pidController.calculate(currentAngle, targetAngle.getDegrees()),
        //     -Settings.kMaxAngularVelocity.getDegrees(),
        //     Settings.kMaxAngularVelocity.getDegrees()
        // ));
        
        // CommandSwerveDrivetrain.getInstance().applyRequest(() -> {
        //     return RobotContainer.drive.withVelocityX(driver.getDriveTranslation().getX() * RobotContainer.kMaxVelocity)
        //     .withVelocityY(driver.getDriveTranslation().getY() * RobotContainer.kMaxVelocity)
        //     .withRotationalRate((requestedAngularVelocity.getRadians() / 4.0) * RobotContainer.kMaxAngularVelocity);
        // });
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerve();
    }
}
