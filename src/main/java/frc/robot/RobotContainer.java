package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.algaeflywheel.AlgaeRoller;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.auton.AutonMaster;
import frc.robot.coralArm.CoralSubsystem;
import frc.robot.coralator.CoralRollerSubsystem;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.drivetrain.TunerConstants;
import frc.robot.elevator.ElevatorSubsystem;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.operator.OperatorXbox;
import frc.robot.vision.PoseEstimatorSubsystem;
import frc.robot.vision.VisionConfig;

import frc.robot.rushinator.*;
import frc.robot.rushinator.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static double kMaxVelocity = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double kMaxAngularVelocity = RotationsPerSecond.of(1.0).in(RadiansPerSecond);
    
    public static boolean modeFast = true;

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxVelocity * 0.1)
        .withRotationalDeadband(kMaxAngularVelocity * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public static SendableChooser<Command> mAutonChooser;
    AutonMaster mAutonMaster = new AutonMaster();

    public RobotContainer() {
        mAutonChooser = mAutonMaster.getAutonSelector();
        setDefaultCommands();

        ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Chooser");
        autonTab.add(mAutonChooser);
        SmartDashboard.putData(mAutonChooser);

        var mRushPivot = RushinatorPivot.getInstance();

        var mPoseEstimator = PoseEstimatorSubsystem.getInstance();

        // ShuffleboardTab visionTab = Shuffleboard.getTab("Vision Pose Estimator");
        // PoseEstimatorSubsystem.getInstance().addDashboardWidgets(visionTab);
    }

        /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
    //     return CommandSwerveDrivetrain.getInstance().applyRequest(() -> 
    //     RobotContainer.drive.withVelocityX(0.25 * RobotContainer.kMaxVelocity) // Drive forward with negative Y (forward)
    //          .withVelocityY(0.0) // Drive left with negative X (left)
    //          .withRotationalRate(0.0) // Drive counterclockwise with negative X (left)
    //  );
        return mAutonChooser.getSelected();
    }


    public void setDefaultCommands() {
        final var driver = DriverXbox.getInstance();
        final var operator = OperatorXbox.getInstance();

        CommandSwerveDrivetrain.getInstance().setDefaultCommand(
                CommandSwerveDrivetrain.getInstance().applyRequest(() -> {
                    if (modeFast) {
                    return drive.withVelocityX(driver.getDriveTranslation().getX() * kMaxVelocity) // Drive forward with negative Y (forward)
                    .withVelocityY(driver.getDriveTranslation().getY() * kMaxVelocity) // Drive left with negative X (left)
                    .withRotationalRate(driver.getDriveRotation() * kMaxAngularVelocity); // Drive counterclockwise with negative X (left)
                } else {
                    return drive.withVelocityX(driver.getDriveTranslation().getX() * kMaxVelocity * 0.5) // Drive forward with negative Y (forward)
                        .withVelocityY(driver.getDriveTranslation().getY() * kMaxVelocity * 0.5) // Drive left with negative X (left)
                        .withRotationalRate(driver.getDriveRotation() * kMaxAngularVelocity * 0.5); // Drive counterclockwise with negative X (left)
                }
            })
        );

        // RushinatorWrist.getInstance().setDefaultCommand(new HoldWristPosition());
        // RushinatorWrist.getInstance().setDefaultCommand(new ManualWristControl(() -> operator.getLeftY()));
        

        // CommandSwerveDrivetrain.getInstance().setDefaultCommand(
        //     CommandSwerveDrivetrain.getInstance().applyRequest(() -> {
        //         return drive.withVelocityX(driver.getDriveTranslation().getX() * kMaxVelocity) // Drive forward with negative Y (forward)
        //                 .withVelocityY(driver.getDriveTranslation().getY() * kMaxVelocity) // Drive left with negative X (left)
        //                 .withRotationalRate(driver.getDriveRotation() * kMaxAngularVelocity); // Drive counterclockwise with negative X (left)
        //     })
        // );

        // AlgaeRoller.getInstance().setDefaultCommand(new AlgaeRoller.DefaultCommand());
        // ElevatorSubsystem.getInstance().setDefaultCommand(
        //         new ElevatorSubsystem.DefaultCommand(ElevatorSubsystem.getInstance(), operator::getElevatorOutput)
        // );

        // ElevatorSubsystem.getInstance().setDefaultCommand(
        //         new ElevatorSubsystem.VelocityCommand(ElevatorSubsystem.getInstance(), operator::getElevatorOutput)
        // );
        // AlgaeSubsystem.getInstance().setDefaultCommand(
        //     new AlgaeSubsystem.DefaultCommand()
        // );
        // CoralRollerSubsystem.getInstance().setDefaultCommand(new CoralRollerSubsystem.SetVoltageCommand(0));
        // CoralSubsystem.getInstance().setDefaultCommand(new CoralSubsystem.DefaultCommand());
        // AlgaeRoller.getInstance().setDefaultCommand(new AlgaeRoller.SetIndexerVoltagCommand(AlgaeRoller.getInstance(), -5));
        
//        CoralSubsystem.getInstance().setDefaultCommand(new CoralSubsystem.TuningCommand(() -> (driver.getRightX() + 1) / 2.0f));
    }
}
  