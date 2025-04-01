package frc.robot.vision.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.vision.VisionConfig.AlignmentConfig.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  private Pose2d goalPose2d;

  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
  /**
   * Constructs a DriveToPoseCommand
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param goalPose goal pose to drive to
   */
  public AutoAlign(Pose2d targetPose) {
    this(drivetrainSubsystem, poseProvider);
    this.goalPose2d = targetPose;
    // setGoal(targetPose);
  }

  /**
   * Constructs a DriveToPoseCommand with specific motion profile constraints
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param poseProvider provider to call to get the robot pose
   */
  public AutoAlign(
      CommandSwerveDrivetrain drivetrainSubsystem,
      Supplier<Pose2d> poseProvider) {

    // this.drivetrainSubsystem = CommandSwerveDrivetrain.getInstance();
    // this.poseProvider = () -> PoseEstimatorSubsystem.getInstance().getCurrentPose();

    xController = new PIDController(XY_kP, XY_kI, XY_kD);
    xController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));

    yController = new PIDController(XY_kP, XY_kI, XY_kD);
    yController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));

    thetaController = new PIDController(THETA_kP, THETA_kI, THETA_kD);
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

  private ChassisSpeeds applyLimits(ChassisSpeeds speeds){
    Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double magnitude = translation.getNorm();
    if (magnitude < .07) return new ChassisSpeeds(0,0, speeds.omegaRadiansPerSecond);
    Rotation2d angle = translation.getAngle();
    magnitude = Math.min(magnitude, MAX_ALIGN_TRANSLATION_VELOCITY.in(MetersPerSecond));
    return new ChassisSpeeds(magnitude * angle.getCos(), magnitude * angle.getSin(), speeds.omegaRadiansPerSecond);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();

    xSpeed = xController.calculate(robotPose.getX(), this.goalPose2d.getX());
    if (xController.atSetpoint()) {
      xSpeed = 0;
    }

    ySpeed = yController.calculate(robotPose.getY(), this.goalPose2d.getY());
    if (yController.atSetpoint()) {
      ySpeed = 0;
    }

    omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians(), this.goalPose2d.getRotation().getRadians());
    if (thetaController.atSetpoint()) {
      omegaSpeed = 0;
    }

    ChassisSpeeds speeds = applyLimits(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));

    // CommandSwerveDrivetrain.getInstance().applyRequest( () -> 
    //     RobotContainer.drive.withVelocityX(speeds.vxMetersPerSecond)
    //         .withVelocityY(speeds.vyMetersPerSecond)
    //         .withRotationalRate(speeds.omegaRadiansPerSecond)
    // ).execute();
    drivetrainSubsystem.setControl(applyFieldSpeeds.withSpeeds(speeds));
    // drivetrainSubsystem.setControl(
    //     fieldCentricSwerveRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(omegaSpeed));
    SmartDashboard.putNumber("Goal Pose X", this.goalPose2d.getX());
    SmartDashboard.putNumber("Goal Pose Y", this.goalPose2d.getY());
    SmartDashboard.putNumber("Goal Pose Theta", this.goalPose2d.getRotation().getRadians());
    SmartDashboard.putNumber("Current Robot Pose X", robotPose.getX());
    SmartDashboard.putNumber("Current Robot Pose Y", robotPose.getY());
    SmartDashboard.putNumber("Current Robot Pose Theta", robotPose.getRotation().getRadians());
    SmartDashboard.putNumber("X Setpoint", xController.getSetpoint());
    SmartDashboard.putNumber("Y Setpoint", yController.getSetpoint());
    SmartDashboard.putNumber("Theta Setpoint", thetaController.getSetpoint());
    SmartDashboard.putNumber("X Output", xController.calculate(robotPose.getX()));
    SmartDashboard.putNumber("Y Output", yController.calculate(robotPose.getY()));
    SmartDashboard.putNumber("Theta Output", thetaController.calculate(robotPose.getRotation().getRadians()));
    SmartDashboard.putNumber("X Error", xController.getError());
    SmartDashboard.putNumber("Y Error", yController.getError());
    SmartDashboard.putNumber("Theta Error", thetaController.getError());
    SmartDashboard.putBoolean("X at Setpoint", xController.atSetpoint());
    SmartDashboard.putBoolean("Y at Setpoint", yController.atSetpoint());
    SmartDashboard.putBoolean("Theta at Setpoint", thetaController.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setControl(new SwerveRequest.Idle());
  }
}