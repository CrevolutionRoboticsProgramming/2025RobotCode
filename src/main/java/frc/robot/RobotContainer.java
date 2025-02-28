package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.drivetrain.TunerConstants;
// import frc.robot.drivetrain2.Drivetrain;
// import frc.robot.drivetrain2.DrivetrainConfig;
// import frc.robot.drivetrain2.DrivetrainConfig.DriveConstants;
// import frc.robot.drivetrain2.commands.DrivetrainCommands;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //AutonMaster mAutonMaster = new AutonMaster();

    // Gamepads


    /* Subsystems */

    /* Auton Chooser */
    public static SendableChooser<Command> mAutonChooser;

    public RobotContainer() {
        //mAutonChooser = mAutonMaster.getAutonSelector();
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
        // final var driver = DriverXbox.getInstance();

        // CommandSwerveDrivetrain.getInstance().setDefaultCommand(
        //     DrivetrainCommands.drive(
        //         driver.getDriveTranslation().getX() * MaxSpeed, 
        //         driver.getDriveTranslation().getY() * MaxSpeed, 
        //         driver.getDriveRotation() * MaxAngularRate
        //     )
        // );

        CommandSwerveDrivetrain.getInstance().setDefaultCommand(
            CommandSwerveDrivetrain.getInstance().applyRequest(() -> 
                drive.withVelocityX(driver.getDriveTranslation().getX() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(driver.getDriveTranslation().getY() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(driver.getDriveRotation() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Drivetrain.getInstance().setDefaultCommand(DrivetrainCommands.drive(
        //     driver::getDriveTranslation,
        //     driver::getDriveRotation
        // ));
    }
}
  