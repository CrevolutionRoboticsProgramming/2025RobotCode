package frc.robot.vision.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.vision.VisionConfig.AlignmentConfig.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.vision.PoseEstimatorSubsystem;

import java.util.function.Supplier;

/**
 * Command to drive to a pose.
 */
public class AutoAlign extends Command {

  private static final Distance TRANSLATION_TOLERANCE = Inches.of(0.5);
  private static final Angle THETA_TOLERANCE = Degrees.of(1.0);

//   protected static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
//       MAX_ALIGN_TRANSLATION_VELOCITY.in(MetersPerSecond),
//       MAX_ALIGN_TRANSLATION_ACCELERATION.in(MetersPerSecondPerSecond));
//   protected static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
//       MAX_ALIGN_ANGULAR_VELOCITY.in(RadiansPerSecond),
//       MAX_ALIGN_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  double xSpeed;
  double ySpeed;
  double omegaSpeed;

  private final static CommandSwerveDrivetrain drivetrainSubsystem = CommandSwerveDrivetrain.getInstance();
//   private final FieldCentric fieldCentricSwerveRequest = new FieldCentric()
//       .withSteerRequestType(SteerRequestType.MotionMagicExpo)
//       .withDriveRequestType(DriveRequestType.Velocity)
//       .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance); // Always Blue coordinate system for auto drive
  protected final static Supplier<Pose2d> poseProvider = () -> PoseEstimatorSubsystem.getInstance().getCurrentPose();

  /**
   * Constructs a DriveToPoseCommand
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param goalPose goal pose to drive to
   */
  public AutoAlign(Pose2d targetPose) {
    this(drivetrainSubsystem, poseProvider);
    setGoal(targetPose);
  }

  /**
   * Constructs a DriveToPoseCommand with specific motion profile constraints
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param poseProvider provider to call to get the robot pose
   * @param translationConstraints translation motion profile constraints
   * @param omegaConstraints rotation motion profile constraints
   */
  public AutoAlign(
      CommandSwerveDrivetrain drivetrainSubsystem,
      Supplier<Pose2d> poseProvider) {

    // this.drivetrainSubsystem = CommandSwerveDrivetrain.getInstance();
    // this.poseProvider = () -> PoseEstimatorSubsystem.getInstance().getCurrentPose();

    xController = new ProfiledPIDController(X_kP, X_kI, X_kD);
    xController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));

    yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD);
    yController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));

    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE.in(Radians));

    addRequirements(drivetrainSubsystem);
  }

  /**
   * Sets the goal to drive to. This should be set before the command is scheduled.
   * 
   * @param goalPose goal pose
   */
  public void setGoal(Pose2d goalPose) {
    thetaController.setSetpoint(goalPose.getRotation().getRadians());
    xController.setSetpoint(goalPose.getX());
    yController.setSetpoint(goalPose.getY());
  }

  @Override
  public void initialize() {
    var robotPose = poseProvider.get();
    thetaController.reset();
    xController.reset();
    yController.reset();
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();

    xSpeed = xController.calculate(robotPose.getX());
    if (xController.atSetpoint()) {
      xSpeed = 0;
    }

    ySpeed = yController.calculate(robotPose.getY());
    if (yController.atSetpoint()) {
      ySpeed = 0;
    }

    omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atSetpoint()) {
      omegaSpeed = 0;
    }

    CommandSwerveDrivetrain.getInstance().applyRequest( () -> 
        RobotContainer.drive.withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(omegaSpeed)
    ).execute();
    // drivetrainSubsystem.setControl(
    //     fieldCentricSwerveRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(omegaSpeed));
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setControl(new SwerveRequest.Idle());
  }

  private ChassisSpeeds applyLimits(ChassisSpeeds speeds){
    Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double magnitude = translation.getNorm();
    if (magnitude < .07) return new ChassisSpeeds(0,0, speeds.omegaRadiansPerSecond);
    Rotation2d angle = translation.getAngle();
    magnitude = Math.min(magnitude, MAX_ALIGN_TRANSLATION_VELOCITY.in(MetersPerSecond));
    return new ChassisSpeeds(magnitude * angle.getCos(), magnitude * angle.getSin(), speeds.omegaRadiansPerSecond);
  }

}