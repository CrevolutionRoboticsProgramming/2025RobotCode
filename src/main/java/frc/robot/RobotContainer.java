package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.drivetrain.TunerConstants;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final double kMaxVelocity = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double kMaxAngularVelocity = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxVelocity * 0.1)
        .withRotationalDeadband(kMaxAngularVelocity * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public static SendableChooser<Command> mAutonChooser;

    private final AlgaeSubsystem algaeSubsystem = AlgaeSubsystem.getInstance();

    public RobotContainer() {
        setDefaultCommands();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return mAutonChooser.getSelected();
    }

    public void setDefaultCommands() {
        final var driver = DriverXbox.getInstance();
//        CommandSwerveDrivetrain.getInstance().setDefaultCommand(
//            CommandSwerveDrivetrain.getInstance().applyRequest(() ->
//                drive.withVelocityX(driver.getDriveTranslation().getX() * kMaxVelocity) // Drive forward with negative Y (forward)
//                .withVelocityY(driver.getDriveTranslation().getY() * kMaxVelocity) // Drive left with negative X (left)
//                .withRotationalRate(driver.getDriveRotation() * kMaxAngularVelocity) // Drive counterclockwise with negative X (left)
//            )
//        );

        //algaeSubsystem.setDefaultCommand(new AlgaeSubsystem.DefaultCommand(algaeSubsystem, () -> (driver.getRightX() + 1) / 2.0));
        algaeSubsystem.setDefaultCommand(new AlgaeSubsystem.DefaultCommand(algaeSubsystem, () -> 1.0));
    }
}
  