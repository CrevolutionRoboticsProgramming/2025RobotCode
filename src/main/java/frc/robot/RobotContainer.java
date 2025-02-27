package frc.robot;

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
import frc.robot.driver.Driver;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.DrivetrainConfig;
import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;
import frc.robot.drivetrain.commands.DrivetrainCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

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
        final var driver = Driver.getInstance();
        // final var driver = DriverXbox.getInstance();

        Drivetrain.getInstance().setDefaultCommand(DrivetrainCommands.drive(
            driver::getDriveTranslation,
            driver::getDriveRotation
        ));
    }
}
  