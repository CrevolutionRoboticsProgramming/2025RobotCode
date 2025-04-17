// package frc.robot.vision.commands;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.swerve.SwerveModule;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.crevolib.math.Conversions;
// import frc.robot.drivetrain.CommandSwerveDrivetrain;
// import frc.robot.vision.PoseEstimatorSubsystem;
// import static frc.robot.vision.VisionConfig.AlignmentConfig.*;
// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.MetersPerSecond;
// import static edu.wpi.first.units.Units.Radians;

// public class AutoAlignHP extends Command {
//     private final Distance TRANSLATION_TOLERANCE = Inches.of(0.25);
//     private final Angle THETA_TOLERANCE = Degrees.of(1.0);

//     private final static CommandSwerveDrivetrain drivetrainSubsystem = CommandSwerveDrivetrain.getInstance();
//     protected final static Supplier<Pose2d> currentPoseProvider = () -> PoseEstimatorSubsystem.getInstance().getCurrentPose();
//     protected Supplier<Pose2d> targetPoseSupplier;
//     private Pose2d goalPose2d;
//     public final Transform2d robotOffset = new Transform2d(0.3018, 0, Rotation2d.kZero);

//     private final PIDController xController;
//     private final PIDController yController;
//     private final PIDController thetaController;
//     double xSpeed;
//     double ySpeed;
//     double omegaSpeed;

//     private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
//             .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

//     public AutoAlignHP(Supplier<Pose2d> targetPose) {
//         this(drivetrainSubsystem, currentPoseProvider);
//         this.targetPoseSupplier = targetPose;
//     }

//     public AutoAlignHP(CommandSwerveDrivetrain drivetrainSubsystem, Supplier<Pose2d> currentPoseProvider) {
//         xController = new PIDController(XY_kP, XY_kI, XY_kD);
//         xController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));

//         yController = new PIDController(XY_kP, XY_kI, XY_kD);
//         yController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));

//         thetaController = new PIDController(THETA_kP, THETA_kI, THETA_kD);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);
//         thetaController.setTolerance(THETA_TOLERANCE.in(Radians));
//         addRequirements(drivetrainSubsystem);
//     }

//     public void setGoal(Pose2d goalPose) {
//         thetaController.setSetpoint(goalPose.getRotation().getRadians());
//         xController.setSetpoint(goalPose.getX());
//         yController.setSetpoint(goalPose.getY());
//     }

//     private ChassisSpeeds applyLimits(ChassisSpeeds speeds){
//         Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
//         double magnitude = translation.getNorm();
//         if (magnitude < .07) return new ChassisSpeeds(0,0, speeds.omegaRadiansPerSecond);
//         Rotation2d angle = translation.getAngle();
//         magnitude = Math.min(magnitude, MAX_ALIGN_TRANSLATION_VELOCITY.in(MetersPerSecond));
//         return new ChassisSpeeds(magnitude * angle.getCos(), magnitude * angle.getSin(), speeds.omegaRadiansPerSecond);
//     }

//     @Override
//     public void initialize() {
//         xController.reset();
//         yController.reset();
//         thetaController.reset();
//     }

//     @Override
//     public void execute() {
//         goalPose2d = targetPoseSupplier.get();
//         goalPose2d = Conversions.rotatePose(goalPose2d.transformBy(robotOffset), Rotation2d.kZero);

//         var robotPose = currentPoseProvider.get();

//         xSpeed = xController.calculate(robotPose.getX(), this.goalPose2d.getX());
//         if (xController.atSetpoint()) {
//         xSpeed = 0;
//         }

//         ySpeed = yController.calculate(robotPose.getY(), this.goalPose2d.getY());
//         if (yController.atSetpoint()) {
//         ySpeed = 0;
//         }

//         omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians(), this.goalPose2d.getRotation().getRadians());
//         if (thetaController.atSetpoint()) {
//         omegaSpeed = 0;
//         }

//         ChassisSpeeds speeds = applyLimits(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
//         drivetrainSubsystem.setControl(applyFieldSpeeds.withSpeeds(speeds));

//         SmartDashboard.putNumber("Goal Pose X - HP Align", this.goalPose2d.getX());
//         SmartDashboard.putNumber("Goal Pose Y - HP Align", this.goalPose2d.getY());
//         SmartDashboard.putNumber("Goal Pose Theta - HP Align", this.goalPose2d.getRotation().getRadians());
//         SmartDashboard.putNumber("Current Robot Pose X - HP Align", robotPose.getX());
//         SmartDashboard.putNumber("Current Robot Pose Y - HP Align", robotPose.getY());
//         SmartDashboard.putNumber("Current Robot Pose Theta - HP Align", robotPose.getRotation().getRadians());
//         SmartDashboard.putNumber("X Setpoint - HP Align", xController.getSetpoint());
//         SmartDashboard.putNumber("Y Setpoint - HP Align", yController.getSetpoint());
//         SmartDashboard.putNumber("Theta Setpoint - HP Align", thetaController.getSetpoint());
//         SmartDashboard.putNumber("X Output - HP Align", xController.calculate(robotPose.getX()));
//         SmartDashboard.putNumber("Y Output - HP Align", yController.calculate(robotPose.getY()));
//         SmartDashboard.putNumber("Theta Output - HP Align", thetaController.calculate(robotPose.getRotation().getRadians()));
//         SmartDashboard.putNumber("X Error - HP Align", xController.getError());
//         SmartDashboard.putNumber("Y Error - HP Align", yController.getError());
//         SmartDashboard.putNumber("Theta Error - HP Align", thetaController.getError());
//         SmartDashboard.putBoolean("X at Setpoint - HP Align", xController.atSetpoint());
//         SmartDashboard.putBoolean("Y at Setpoint - HP Align", yController.atSetpoint());
//         SmartDashboard.putBoolean("Theta at Setpoint - HP Align", thetaController.atSetpoint());
//     }

//     @Override
//     public boolean isFinished() {
//         return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrainSubsystem.setControl(new SwerveRequest.Idle());
//     }
// }
