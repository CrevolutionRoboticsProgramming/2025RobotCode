package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.algaeflywheel.AlgaeRoller;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.drivetrain.TunerConstants;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.operator.OperatorXbox;
import frc.robot.subsystems.CoralRollerSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static double kMaxVelocity = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double kMaxAngularVelocity = RotationsPerSecond.of(2.5).in(RadiansPerSecond);
    
    public static boolean modeFast = true;

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxVelocity * 0.1)
        .withRotationalDeadband(kMaxAngularVelocity * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public static SendableChooser<Command> mAutonChooser;

    public RobotContainer() {
        setDefaultCommands();
    }

        /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return CommandSwerveDrivetrain.getInstance().applyRequest(() -> 
        RobotContainer.drive.withVelocityX(0.25 * RobotContainer.kMaxVelocity) // Drive forward with negative Y (forward)
             .withVelocityY(0.0) // Drive left with negative X (left)
             .withRotationalRate(0.0) // Drive counterclockwise with negative X (left)
     );
    }


    public void setDefaultCommands() {
        final var driver = DriverXbox.getInstance();
        final var operator = OperatorXbox.getInstance();

        CommandSwerveDrivetrain.getInstance().setDefaultCommand(
            CommandSwerveDrivetrain.getInstance().applyRequest(() -> {
                return drive.withVelocityX(driver.getDriveTranslation().getX() * kMaxVelocity) // Drive forward with negative Y (forward)
                        .withVelocityY(driver.getDriveTranslation().getY() * kMaxVelocity) // Drive left with negative X (left)
                        .withRotationalRate(driver.getDriveRotation() * kMaxAngularVelocity); // Drive counterclockwise with negative X (left)
            })
        );

        AlgaeRoller.getInstance().setDefaultCommand(new AlgaeRoller.DefaultCommand());
        ElevatorSubsystem.getInstance().setDefaultCommand(
                new ElevatorSubsystem.DefaultCommand(ElevatorSubsystem.getInstance(), operator::getElevatorOutput)
        );
        AlgaeSubsystem.getInstance().setDefaultCommand(
            new AlgaeSubsystem.DefaultCommand()
        );
        CoralRollerSubsystem.getInstance().setDefaultCommand(new CoralRollerSubsystem.SetVoltageCommand(0));
        CoralSubsystem.getInstance().setDefaultCommand(new CoralSubsystem.DefaultCommand());
        AlgaeRoller.getInstance().setDefaultCommand(new InstantCommand(() -> AlgaeRoller.getInstance().setIndexerVoltage(-6)));
//        CoralSubsystem.getInstance().setDefaultCommand(new CoralSubsystem.TuningCommand(() -> (driver.getRightX() + 1) / 2.0f));
    }
}
  