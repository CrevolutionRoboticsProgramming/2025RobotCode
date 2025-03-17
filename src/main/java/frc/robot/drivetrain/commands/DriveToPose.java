// package frc.robot.drivetrain.commands;

// import static com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType.MotionMagicExpo;
// import static com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity;
// import static frc.robot.drivetrain.TunerConstants.DriveCommandsConstants;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.drivetrain.CommandSwerveDrivetrain;

// public class DriveToPose extends Command{
    
//     private final ProfiledPIDController xController;
//     private final ProfiledPIDController yController;
//     private final ProfiledPIDController omegaController;
//     private final ForwardPerspectiveValue perspectiveValue;

//     private final CommandSwerveDrivetrain swerveDrivetrain;
//     private final SwerveRequest.FieldCentric fieldCentricSwerveRequest;

//     private final Supplier<Pose2d> poseSupplier;
//     // = new SwerveRequest.FieldCentric()
//     // .withSteerRequestType(MotionMagicExpo)
//     // .withDriveRequestType(Velocity)
//     // .withForwardPerspective(perspectiveValue);

    
//     /**
//     * Constructs a DriveToPoseCommand
//     * 
//     * @param drivetrainSubsystem drivetrain subsystem
//     * @param goalPose goal pose to drive to
//     */
//     public DriveToPoseCommand(CommandSwerveDrivetrain drivetrainSubsystem, Supplier<Pose2d> poseProvider) {
//         this(drivetrainSubsystem, poseProvider, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS);
//   }
// }
